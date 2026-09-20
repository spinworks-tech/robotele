# Cloning the XGO Pi CM4's microSD card (Ubuntu)

The XGO's Raspberry Pi CM4 boots from a removable microSD card (confirmed via
`dmesg`: `mmc0: new ultra high speed DDR50 SDHC card` — a genuine SD card, not
soldered eMMC). This runbook clones the current card onto a larger one
(e.g. a 128GB SanDisk Ultra A1) using an Ubuntu machine with a card reader.

**This process is non-destructive to the original card** — every step only
*reads* from it, never writes. If anything goes wrong after swapping in the
new card, power off and put the original card back exactly as it was.

## 0. Shut the Pi down cleanly before removing the card

Don't pull the card while the Pi is powered on — that risks filesystem
corruption on the very card you're about to clone.

```bash
ssh pi@<robot-ip> "sudo shutdown -h now"
```

Wait for the Pi's LEDs to stop blinking (or a fixed ~15s if you can't see
them) before physically removing the microSD card.

## 1. Identify the SD card device — carefully

Insert **only the source (original) card** into your Ubuntu machine's reader
first, one card at a time, so there's no ambiguity about which device is
which.

```bash
lsblk
```

Look for a device sized close to what you saw on the Pi itself (29.3G from
`lsblk` run there) with two partitions (a small `~256M` boot partition and a
larger root partition), e.g.:

```
sdb           8:16   1  29.3G  0 disk
├─sdb1        8:17   1   256M  0 part
└─sdb2        8:18   1  14.6G  0 part
```

**Double- and triple-check the device name (`/dev/sdX`) before continuing.**
Getting this wrong and pointing `dd` at your Ubuntu machine's own disk will
destroy its filesystem with no warning and no confirmation prompt. If you're
at all unsure, run `lsblk` again after unplugging the reader and compare
which device disappeared.

Unmount any auto-mounted partitions from that device (replace `sdb` with your
actual device):

```bash
sudo umount /dev/sdb1 /dev/sdb2 2>/dev/null
```

## 2. Clone the source card to an image file

```bash
sudo dd if=/dev/sdb of=~/xgo-pi-backup.img bs=4M status=progress conv=fsync
sync
```

- `bs=4M` — reasonable block size for a fast, reliable copy on a card reader.
- `status=progress` — shows live progress (needs a moderately recent
  `coreutils`; Ubuntu's is fine).
- `conv=fsync` — forces the write to actually land on disk before `dd`
  reports done, not just sit in a cache.

This creates a ~29.3GB image file (the source card's reported size, not the
destination card's size) at `~/xgo-pi-backup.img`. It'll take a while
depending on your reader/card speed — expect tens of minutes, not seconds.

Once done, safely remove the original card:

```bash
sudo eject /dev/sdb
```

## 3. Write the image to the new 128GB card

Swap in the new SanDisk 128GB card, then find its device name the same
careful way as step 1 (`lsblk` before/after inserting it — it will likely get
a different device letter than the original card did, e.g. `sdb` again if
nothing else changed, but verify, don't assume).

```bash
lsblk
sudo umount /dev/sdb1 /dev/sdb2 2>/dev/null
sudo dd if=~/xgo-pi-backup.img of=/dev/sdb bs=4M status=progress conv=fsync
sync
sudo eject /dev/sdb
```

**Same warning as step 1 applies here, in the other direction**: writing to
the wrong device will destroy whatever's on it. Confirm `/dev/sdb` (or
whatever it shows as) is genuinely the 128GB card before running `dd`.

## 4. (Optional) Verify the clone

Slow (reads the full ~29.3GB twice) but confirms bit-for-bit correctness if
you want extra confidence before relying on the new card:

```bash
sha256sum ~/xgo-pi-backup.img
sudo dd if=/dev/sdb bs=4M count=7500 status=progress | sha256sum
```

(`count=7500` at `bs=4M` ≈ 29.3GB — reads back only the portion matching the
image size, not the full 128GB card, since the rest is still unpartitioned
space at this point.) The two checksums should match.

## 5. Boot the Pi from the new card and expand the filesystem

Insert the 128GB card into the Pi's CM4 carrier board and power it on.

```bash
ssh pi@<robot-ip>
df -h /
```

At this point `df -h /` will still show the **original ~14.6G partition
size** — `dd` cloned the partition table as-is, it doesn't know the
destination card is bigger. Grow it to fill the new card:

```bash
sudo raspi-config --expand-rootfs
sudo reboot
```

After the reboot, confirm the extra space landed:

```bash
ssh pi@<robot-ip> "df -h /"
```

`Avail` should now show close to 128GB instead of the ~350MB-3.5GB range we
were fighting with on the original card.

## 6. Fallback: reverting to the original card

If anything looks wrong at any point after step 3 — power off the Pi, remove
the 128GB card, and reinsert the original card. Nothing in this process ever
wrote to the original card, so it's untouched and the Pi will boot exactly as
it did before any of this started.
