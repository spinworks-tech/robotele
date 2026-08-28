//! Dev-only mTLS certificate generator for RoboProtocol.
//!
//! Generates a self-signed CA plus robot/operator leaf certs signed by it.
//! This is explicitly a development convenience -- no revocation, no
//! rotation, no production PKI. Re-running overwrites existing output
//! unless `--no-overwrite` is passed.
//!
//! **Algorithm: ECDSA P-256, not Ed25519 (documented v0 deviation).**
//! DESIGN.md §4 specifies Ed25519 as the primary signature algorithm,
//! with "ECDSA P-256 permitted only where HSM/TPM hardware mandates a
//! NIST curve." That carve-out doesn't literally apply here, but v0 uses
//! P-256 anyway for a different, empirically-discovered reason: `quiche`
//! 0.22.0's vendored BoringSSL build fails to load Ed25519 PKCS8 private
//! keys via `load_priv_key_from_pem_file` (`Error::TlsFail`), regardless
//! of PKCS8 v1 vs v2 encoding -- confirmed by testing both against a
//! running `robot-edge`. The same `rcgen`-generated ECDSA P-256 key loads
//! without issue. Revisit Ed25519 once that's root-caused (a different
//! quiche/BoringSSL build, or the `boringssl-boring-crate`/`openssl`
//! quiche feature instead of the default vendored one) -- this is a
//! build/library gap, not a spec change.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use rcgen::{
    BasicConstraints, CertificateParams, DistinguishedName, DnType, IsCa, KeyPair,
    KeyUsagePurpose, SanType,
};

struct Args {
    out_dir: PathBuf,
    robot_sans: Vec<String>,
    no_overwrite: bool,
}

impl Args {
    fn parse() -> Result<Self> {
        let mut out_dir = PathBuf::from("certs");
        let mut robot_sans = Vec::new();
        let mut no_overwrite = false;

        let mut it = std::env::args().skip(1);
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--out-dir" => {
                    out_dir = PathBuf::from(it.next().context("--out-dir needs a value")?);
                }
                "--robot-san" => {
                    robot_sans.push(it.next().context("--robot-san needs a value")?);
                }
                "--no-overwrite" => no_overwrite = true,
                "-h" | "--help" => {
                    println!(
                        "Usage: dev-certs [--out-dir DIR] [--robot-san IP-OR-DNS]... [--no-overwrite]\n\n\
                         Generates a dev-only ECDSA P-256 CA + robot/operator mTLS certs for RoboProtocol."
                    );
                    std::process::exit(0);
                }
                other => anyhow::bail!("unrecognized argument: {other} (see --help)"),
            }
        }
        Ok(Self { out_dir, robot_sans, no_overwrite })
    }
}

fn main() -> Result<()> {
    let args = Args::parse()?;

    if args.no_overwrite && args.out_dir.exists() {
        anyhow::bail!("{:?} already exists and --no-overwrite was passed", args.out_dir);
    }

    let ca_dir = args.out_dir.join("dev-ca");
    let robot_dir = args.out_dir.join("robot");
    let operator_dir = args.out_dir.join("operator");
    for dir in [&ca_dir, &robot_dir, &operator_dir] {
        fs::create_dir_all(dir).with_context(|| format!("creating {dir:?}"))?;
    }

    let (ca_key, ca_cert) = make_ca("RoboProtocol Dev CA")?;
    write_pem(&ca_dir.join("ca.key"), &ca_key.serialize_pem())?;
    write_pem(&ca_dir.join("ca.crt"), &ca_cert.pem())?;

    let mut robot_sans = vec!["robot-edge".to_string(), "localhost".to_string(), "127.0.0.1".to_string()];
    robot_sans.extend(args.robot_sans);
    let (robot_key, robot_cert) = make_leaf("RoboProtocol Robot Edge", &robot_sans, &ca_cert, &ca_key)?;
    write_pem(&robot_dir.join("robot.key"), &robot_key.serialize_pem())?;
    write_pem(&robot_dir.join("robot.crt"), &robot_cert.pem())?;

    let operator_sans = vec!["operator-console".to_string(), "localhost".to_string(), "127.0.0.1".to_string()];
    let (operator_key, operator_cert) = make_leaf("RoboProtocol Operator Console", &operator_sans, &ca_cert, &ca_key)?;
    write_pem(&operator_dir.join("operator.key"), &operator_key.serialize_pem())?;
    write_pem(&operator_dir.join("operator.crt"), &operator_cert.pem())?;

    println!("Generated dev-only ECDSA P-256 CA + leaf certs under {:?}", args.out_dir);
    println!("  {:?}", ca_dir.join("ca.crt"));
    println!("  {:?} / {:?}", robot_dir.join("robot.crt"), robot_dir.join("robot.key"));
    println!("  {:?} / {:?}", operator_dir.join("operator.crt"), operator_dir.join("operator.key"));
    println!("Dev-only: self-signed CA, no revocation/rotation. Do not use in production.");
    Ok(())
}

fn make_ca(common_name: &str) -> Result<(KeyPair, rcgen::Certificate)> {
    let key = KeyPair::generate_for(&rcgen::PKCS_ECDSA_P256_SHA256)?;
    let mut params = CertificateParams::new(Vec::<String>::new())?;
    params.distinguished_name = dn(common_name);
    params.is_ca = IsCa::Ca(BasicConstraints::Unconstrained);
    params.key_usages = vec![KeyUsagePurpose::KeyCertSign, KeyUsagePurpose::CrlSign];
    let cert = params.self_signed(&key)?;
    Ok((key, cert))
}

fn make_leaf(
    common_name: &str,
    sans: &[String],
    ca_cert: &rcgen::Certificate,
    ca_key: &KeyPair,
) -> Result<(KeyPair, rcgen::Certificate)> {
    let key = KeyPair::generate_for(&rcgen::PKCS_ECDSA_P256_SHA256)?;
    let mut params = CertificateParams::new(sans.to_vec())?;
    params.distinguished_name = dn(common_name);
    params.is_ca = IsCa::NoCa;
    params.key_usages = vec![KeyUsagePurpose::DigitalSignature];
    params.subject_alt_names = sans
        .iter()
        .map(|s| {
            s.parse::<std::net::IpAddr>()
                .map(SanType::IpAddress)
                .unwrap_or_else(|_| SanType::DnsName(s.clone().try_into().expect("valid DNS name")))
        })
        .collect();
    let cert = params.signed_by(&key, ca_cert, ca_key)?;
    Ok((key, cert))
}

fn dn(common_name: &str) -> DistinguishedName {
    let mut dn = DistinguishedName::new();
    dn.push(DnType::CommonName, common_name);
    dn
}

fn write_pem(path: &Path, contents: &str) -> Result<()> {
    fs::write(path, contents).with_context(|| format!("writing {path:?}"))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if path.extension().is_some_and(|e| e == "key") {
            fs::set_permissions(path, fs::Permissions::from_mode(0o600))
                .with_context(|| format!("restricting permissions on {path:?}"))?;
        }
    }
    Ok(())
}
