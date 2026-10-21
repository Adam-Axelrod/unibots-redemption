# Remote access to the Pi (Tailscale + SSH)

The Pi lives on King's College London eduroam, which blocks incoming connections, so we
reach it over [Tailscale](https://tailscale.com). The Pi is on Adam's tailnet
and **shared** with Jack and Sami, so each person keeps their own Tailscale
account and sees only the Pi, not each other's devices.

Everyone logs in as the Pi's single `pi` user, each with their own SSH key.
When it is all set up, everyone connects the same way:

```bash
ssh rpi5
```

The Pi itself is called `raspberrypi`, the default name, on the Pi and on
Tailscale. `rpi5` is only a nickname that each person sets up in their own
`~/.ssh/config`.

| Part | Who | Where |
|---|---|---|
| [1. Set up the Pi](#1-set-up-the-pi) | Adam | on the Pi, with monitor + keyboard |
| [2. Adam's laptop](#2-adams-laptop) | Adam | laptop |
| [3. Collaborator's laptop](#3-collaborators-laptop) | Jack, Sami | their laptops |
| [4. Add a collaborator](#4-add-a-collaborator) | Adam | Pi + Tailscale admin console |
| [Sharing the `pi` account](#sharing-the-pi-account) | everyone | on the Pi |

---

## 1. Set up the Pi

Do this with a monitor and keyboard attached. Assumes Raspberry Pi OS
Bookworm or later, which uses NetworkManager, and that the user created in the
setup wizard is `pi`.

### 1.1 Keep the name `raspberrypi`

Don't rename the Pi. On Raspberry Pi OS Trixie, **cloud-init** sets the
hostname from `/boot/firmware/user-data` on every boot, so a rename with
`hostnamectl` or Raspberry Pi Configuration is undone at the next reboot.
Leaving it as `raspberrypi` avoids that, and the `rpi5` nickname lives in
everyone's SSH config instead.

### 1.2 Turn on SSH

```bash
sudo systemctl enable --now ssh
```

### 1.3 Join the tailnet

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname=raspberrypi
```

`--hostname=raspberrypi` pins the Tailscale name, so it doesn't change even if
the Pi's hostname does. Everyone's SSH config depends on that name. To pin it
on a Pi that is already on the tailnet, run
`sudo tailscale set --hostname=raspberrypi`.

`tailscale up` prints a login URL. Open it, either in the Pi's browser or on
your phone, and sign in with **your** Tailscale account. Then confirm:

```bash
tailscale status      # raspberrypi should be listed with a 100.x.y.z address
tailscale ip -4
```

In the [admin console](https://login.tailscale.com/admin/machines), find
`raspberrypi`, open the `…` menu, and choose **Disable key expiry**. If you
skip this, the Pi drops off the tailnet after about 180 days and needs a
keyboard to log back in.

If the machine shows up as `raspberrypi-1` (because an old `raspberrypi`
already exists), delete the old one and rename the new one to `raspberrypi`
from the same `…` menu.

### 1.4 Connect to eduroam (King's College London)

These are King's settings, taken from King's official profile on
[eduroam CAT](https://cat.eduroam.org) ("King's College London" → "KCL
Eduroam"). If they stop working, check there first, or contact the King's IT
Service Desk (8888@kcl.ac.uk, 020 7848 8888).

| Setting | Value |
|---|---|
| Security | WPA & WPA2 Enterprise |
| Authentication | Protected EAP (PEAP) |
| Anonymous identity | *(leave blank)* |
| Domain | `login.kcl.ac.uk` |
| CA certificate | `/etc/NetworkManager/kcl-eduroam-ca.pem` (step 1 below) |
| PEAP version | Automatic |
| Inner authentication | MSCHAPv2 |
| Username | your K number with the domain, e.g. `k1234567@kcl.ac.uk` |
| Password | your King's password |

#### 1. Install King's certificate

The CA certificate lets the Pi check that it's talking to King's real login
server, `login.kcl.ac.uk`. Without it, a fake "eduroam" hotspot can collect your
King's password. The repo includes it as
[`docs/kcl-eduroam-ca.pem`](kcl-eduroam-ca.pem): Sectigo Public Server
Authentication Root R46 plus its intermediate, extracted from King's CAT
profile. From your clone of the repo, copy it to where NetworkManager can
always read it:

```bash
sudo install -m 644 docs/kcl-eduroam-ca.pem /etc/NetworkManager/kcl-eduroam-ca.pem
```

#### 2. Connect from the desktop

Click the **Wi-Fi icon** in the top-right of the desktop, choose **eduroam**,
and fill in the dialog from the table above. For the CA certificate, browse to
`/etc/NetworkManager/kcl-eduroam-ca.pem`. Don't choose "No CA certificate is
required". Click **Connect**.

If you already connected without the certificate, that's fine: step 3 adds it.

#### 3. Make it work with nobody there

A Wi-Fi connection made from the desktop keeps its password in the desktop
**keyring**, and only for the user who made it. After a reboot, the Pi won't
join eduroam until someone unlocks the keyring at the screen, so a headless Pi
just stays offline. King's official CAT installer does the same, so don't use
it on the Pi without also doing this step.

This command stores the password with the system instead, and applies all of
King's settings in one go. Your password is read without echoing so that it
doesn't end up in the shared `pi` shell history.

```bash
read -rsp "King's password: " KCL_PW; echo
sudo nmcli connection modify eduroam \
  802-1x.eap peap \
  802-1x.phase2-auth mschapv2 \
  802-1x.identity "k1234567@kcl.ac.uk" \
  802-1x.ca-cert /etc/NetworkManager/kcl-eduroam-ca.pem \
  802-1x.domain-match login.kcl.ac.uk \
  802-1x.password-flags 0 \
  802-1x.password "$KCL_PW" \
  connection.permissions "" \
  connection.autoconnect yes
unset KCL_PW
sudo nmcli connection up eduroam
```

Put your own K number in `802-1x.identity`. If the connection isn't called
`eduroam`, use the name shown by `nmcli connection show`.

Check it:

```bash
nmcli -g 802-1x.password-flags,connection.permissions connection show eduroam
# should print exactly:  0:
ping -c3 tailscale.com
```

**The real test:** unplug the monitor and keyboard, run `sudo reboot`, and
`ssh rpi5` from your laptop a minute later. Tailscale starts at boot by itself,
so if that works, the Pi will come back on its own after power cuts.

> ⚠️ Your King's password is now stored in
> `/etc/NetworkManager/system-connections/eduroam.nmconnection`, readable by
> root. See [Trust](#trust).

### 1.5 Note your Pi's full Tailscale name

Collaborators need the Pi's full name, because a shared device's short name
does not resolve in their tailnet. Run:

```bash
tailscale status --json | grep -m1 DNSName
# "DNSName": "raspberrypi.tail6c0787.ts.net."
```

Ours is **`raspberrypi.tail6c0787.ts.net`** (drop the trailing dot). That is what you
give Jack and Sami.

---

## 2. Adam's laptop

1. Install Tailscale from <https://tailscale.com/download> and sign in with the
   same account as the Pi.
2. Add this to `~/.ssh/config`:

   ```
   Host rpi5
       HostName raspberrypi
       User pi
       IdentityFile ~/.ssh/id_ed25519
       IdentitiesOnly yes
   ```

   `Host rpi5` is the nickname you type. `HostName raspberrypi` is the Pi's
   Tailscale name, which works on its own because MagicDNS is on by default
   in your own tailnet.

   `IdentitiesOnly yes` matters if your SSH agent holds several keys (check
   with `ssh-add -l`). Without it, SSH offers every key, the Pi counts each
   one as a failed login, and it starts closing the connection with
   `Connection closed by 100.x.y.z port 22` before it ever asks for a password.

3. Copy your key across so you aren't asked for a password every time:

   ```bash
   ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519   # make a key if you have none
   ssh-copy-id -i ~/.ssh/id_ed25519.pub rpi5           # asks for pi's password once
   ssh rpi5
   ```

---

## 3. Collaborator's laptop

For Jack and Sami. Do this before Adam does [part 4](#4-add-a-collaborator).

1. Install Tailscale from <https://tailscale.com/download> and sign in with
   **your own** account. Tell Adam the email address you signed in with.
2. Send Adam your **public** key, the output of:

   ```bash
   ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519   # make a key if you have none
   cat ~/.ssh/id_ed25519.pub
   ```

   It's one line starting `ssh-ed25519`. Public keys are safe to send in a
   message. Never send the file without `.pub`, which is your private key.

3. When Adam's share invite arrives by email, accept it. `raspberrypi` then
   appears in `tailscale status`.
4. Add this to `~/.ssh/config`:

   ```
   Host rpi5
       HostName raspberrypi.tail6c0787.ts.net
       User pi
       IdentityFile ~/.ssh/id_ed25519
       IdentitiesOnly yes
   ```

   `rpi5` is just the nickname you type. `HostName` must be the Pi's full
   Tailscale name, because a shared device's short name doesn't resolve in
   your tailnet.

5. Once Adam says your key is added, log in:

   ```bash
   ssh rpi5
   ```

From now on, `ssh rpi5` works from anywhere, on campus or off, as long as
Tailscale is running on your laptop. Before you do anything else on the Pi,
read [Sharing the `pi` account](#sharing-the-pi-account).

---

## 4. Add a collaborator

Do this once for Jack and once for Sami, after they've done steps 1 and 2 of
[part 3](#3-collaborators-laptop).

### 4.1 Add their key to the Pi

From your laptop, paste their public key between the quotes:

```bash
ssh rpi5 'echo "ssh-ed25519 AAAA...their key... jack@laptop" >> ~/.ssh/authorized_keys'
```

Use `>>`, not `>`: a single `>` replaces the file and locks everyone else out.

### 4.2 Share the Pi with them

1. In the [admin console](https://login.tailscale.com/admin/machines), open
   `raspberrypi` → `…` → **Share…**
2. Invite them **by email**, using the address they sign in to Tailscale with.
3. Send them the Pi's full name from [step 1.5](#15-note-your-pis-full-tailscale-name).

They can reach only the Pi. The Pi can't reach their laptops, and nothing else
on your tailnet is visible to them.

### Removing someone

1. Revoke the share in the same **Share…** dialog. This alone cuts off their
   access.
2. Delete their line from `~/.ssh/authorized_keys` on the Pi. While you're
   there, check that every remaining line is a key you recognise.

---

## Sharing the `pi` account

Everyone is the same user, so anything one person changes, everyone gets. Two
rules keep that from causing trouble.

**One clone each.** Never work in someone else's checkout. Switching branches
or leaving uncommitted changes there changes the code under them too. Keep
your own copy in a folder named after you:

```bash
mkdir -p ~/jack && cd ~/jack
git clone https://github.com/Adam-Axelrod/unibots-redemption.git
cd unibots-redemption
git config user.name "Jack ..."         # no --global: this clone only
git config user.email "jack@..."
```

Without the two `git config` lines, commits made on the Pi are credited to
whoever set the global name.

**Ask before changing shared things.** `~/.bashrc`, installed packages,
system settings and anything under `/etc` affect everyone. Mention it in the
group chat before changing them.

---

## Optional: turn off password logins

Once **everyone** can log in with a key, you can turn off passwords.
Test in a second terminal before closing the first, or you can lock yourself
out:

```bash
echo 'PasswordAuthentication no' | sudo tee /etc/ssh/sshd_config.d/10-no-passwords.conf
sudo systemctl reload ssh
```

---

## Trust

`pi` has passwordless `sudo`, so anyone who can log in has full control of
the Pi. That includes reading the King's password stored for eduroam in step 1.4, and adding
their own keys or accounts. Only share the Pi with people you'd trust with
that password. Better still, ask the King's IT Service Desk whether they offer separate
credentials or a separate network for devices, and use those instead of your
personal login.

## Troubleshooting

| Symptom | Check |
|---|---|
| `Could not resolve hostname raspberrypi` | Is Tailscale running on your laptop? Does `tailscale status` list the Pi as `raspberrypi`? If it shows another name, `ssh pi@100.x.y.z` and run `sudo tailscale set --hostname=raspberrypi`. |
| `Could not resolve hostname rpi5` | Your `~/.ssh/config` has no `Host rpi5` block, or it has a typo. `rpi5` only exists there. |
| `Connection closed by 100.x.y.z port 22`, no password prompt | Your agent offered too many keys. Add `IdentitiesOnly yes` to the `rpi5` block, wait a minute, and try again. |
| `raspberrypi` shows as offline in `tailscale status` | The Pi has no internet. Check eduroam with a keyboard: `nmcli connection show --active`, and the [reconnect check](#3-make-it-work-with-nobody-there). |
| Collaborator can't see `raspberrypi` | They accepted the invite with a different email from the one their Tailscale account uses. Re-share to the right address. |
| Connection works but is slow | Eduroam is blocking direct UDP, so Tailscale is using a relay. `tailscale ping raspberrypi` shows `via DERP`. Fine for SSH. |
| `Permission denied (publickey)` | Your key isn't in `~/.ssh/authorized_keys` on the Pi. Send your `~/.ssh/id_ed25519.pub` to Adam again. |
