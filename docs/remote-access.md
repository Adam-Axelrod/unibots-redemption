# Remote access to the Pi (Tailscale + SSH)

The Pi lives on university eduroam, which blocks incoming connections, so we
reach it over [Tailscale](https://tailscale.com). The Pi is on Adam's tailnet
and **shared** with Jack and Sami, so each person keeps their own Tailscale
account and sees only the Pi, not each other's devices.

When it is all set up, everyone connects the same way:

```bash
ssh rpi5
```

| Part | Who | Where |
|---|---|---|
| [1. Set up the Pi](#1-set-up-the-pi) | Adam | on the Pi, with monitor + keyboard |
| [2. Adam's laptop](#2-adams-laptop) | Adam | laptop |
| [3. Add a collaborator](#3-add-a-collaborator) | Adam | Pi + Tailscale admin console |
| [4. Collaborator's laptop](#4-collaborators-laptop) | Jack, Sami | their laptops |

---

## 1. Set up the Pi

Do this with a monitor and keyboard attached. Assumes Raspberry Pi OS
Bookworm or later, which uses NetworkManager.

### 1.1 Name it `rpi5`

The hostname becomes the Tailscale name, so set it **before** joining the
tailnet:

```bash
sudo hostnamectl set-hostname rpi5
sudo sed -i 's/^127\.0\.1\.1.*/127.0.1.1\trpi5/' /etc/hosts
```

### 1.2 Turn on SSH

```bash
sudo systemctl enable --now ssh
```

### 1.3 Join the tailnet

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

`tailscale up` prints a login URL. Open it, either in the Pi's browser or on
your phone, and sign in with **your** Tailscale account. Then confirm:

```bash
tailscale status      # rpi5 should be listed with a 100.x.y.z address
tailscale ip -4
```

In the [admin console](https://login.tailscale.com/admin/machines), find
`rpi5`, open the `…` menu, and choose **Disable key expiry**. If you skip this,
the Pi drops off the tailnet after about 180 days and needs a keyboard to log
back in.

If the machine shows up as `rpi5-1` (because an old `rpi5` already exists),
delete the old one and rename the new one to `rpi5` from the same `…` menu.

### 1.4 Connect to eduroam

Eduroam is WPA2-Enterprise, which you can't set up from the desktop Wi-Fi menu
or the Pi Imager. Use `nmcli`. The EAP method, CA certificate, and domain differ
between universities, so get them from your university's IT "eduroam on Linux"
page. A typical PEAP setup looks like this:

```bash
sudo nmcli connection add type wifi con-name eduroam ifname wlan0 ssid eduroam \
  wifi-sec.key-mgmt wpa-eap \
  802-1x.eap peap 802-1x.phase2-auth mschapv2 \
  802-1x.identity "you@university.ac.uk" \
  802-1x.password "YOUR_PASSWORD" \
  802-1x.ca-cert /path/to/university-ca.pem \
  802-1x.domain-suffix-match radius.university.ac.uk \
  connection.autoconnect yes
sudo nmcli connection up eduroam
```

Check that it works with `ping -c3 tailscale.com` and `tailscale status`.
Tailscale starts at boot by itself, so nothing else is needed.

> ⚠️ This stores your university password in
> `/etc/NetworkManager/system-connections/eduroam.nmconnection`, readable by
> root. See [Accounts and trust](#accounts-and-trust).

### 1.5 Note your Pi's full Tailscale name

Collaborators need the Pi's full name, because a shared device's short name
does not resolve in their tailnet. Run:

```bash
tailscale status --json | grep -m1 DNSName
# "DNSName": "rpi5.tail1234ab.ts.net."
```

`rpi5.tail1234ab.ts.net` (without the trailing dot) is what you give Jack and
Sami.

---

## 2. Adam's laptop

1. Install Tailscale from <https://tailscale.com/download> and sign in with the
   same account as the Pi.
2. Add this to `~/.ssh/config`, using your username on the Pi (`whoami` on
   the Pi tells you):

   ```
   Host rpi5
       HostName rpi5
       User YOUR_PI_USERNAME
   ```

   `HostName rpi5` works because MagicDNS is on by default in your own tailnet.

3. Copy your key across so you aren't asked for a password every time:

   ```bash
   ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519   # make a key if you have none
   ssh-copy-id rpi5
   ssh rpi5
   ```

---

## 3. Add a collaborator

Do this once for Jack and once for Sami. The commands use `jack`; change the
name for Sami.

### 3.1 Give them an account on the Pi

On the Pi:

```bash
sudo adduser jack                 # set a temporary password, then tell them it
sudo usermod -aG docker jack      # lets them run the docker compose stack
```

Don't add them to `sudo`.

### 3.2 Share the Pi with them

1. In the [admin console](https://login.tailscale.com/admin/machines), open
   `rpi5` → `…` → **Share…**
2. Invite them **by email**, using the address they sign in to Tailscale with.
   They need their own (free) Tailscale account.
3. Send them the Pi's full name from [step 1.5](#15-note-your-pis-full-tailscale-name)
   and the temporary password.

They can reach only the Pi. The Pi can't reach their laptops, and nothing else
on your tailnet is visible to them.

To remove someone later, revoke the share in the same **Share…** dialog, then
run `sudo deluser --remove-home jack` on the Pi.

---

## 4. Collaborator's laptop

For Jack and Sami. You need three things from Adam: the share invite, the Pi's
full name (like `rpi5.tail1234ab.ts.net`), and your temporary password.

1. Install Tailscale from <https://tailscale.com/download> and sign in with
   **your own** account.
2. Accept the share invite from Adam's email. `rpi5` then appears in
   `tailscale status`.
3. Add this to `~/.ssh/config`, using the full name Adam gave you and your
   username on the Pi:

   ```
   Host rpi5
       HostName rpi5.tail1234ab.ts.net
       User jack
   ```

4. Copy your key across, log in, and change the temporary password:

   ```bash
   ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519   # make a key if you have none
   ssh-copy-id rpi5      # asks for the temporary password one last time
   ssh rpi5
   passwd                # on the Pi: replace the temporary password
   ```

From now on, `ssh rpi5` works from anywhere, on campus or off, as long as
Tailscale is running on your laptop.

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

## Accounts and trust

- **Docker group membership gives root access.** Anyone in the `docker` group
  can start a container that mounts `/` and reads any file. That includes the
  eduroam password from step 1.4. Collaborators need Docker to run the stack,
  so only share the Pi with people you'd trust with that password. Better
  still, check whether your university offers separate credentials or a
  separate network for devices, and use those instead of your personal login.
- Each person has their own account, so `last` and `journalctl` show who logged
  in, and you can remove one person without touching the others.

## Troubleshooting

| Symptom | Check |
|---|---|
| `Could not resolve hostname rpi5` | Is Tailscale running on your laptop? Try the `100.x.y.z` address from `tailscale status` in place of the name. |
| `rpi5` shows as offline in `tailscale status` | The Pi has no internet. Check eduroam with a keyboard: `nmcli connection show --active`. |
| Collaborator can't see `rpi5` | They accepted the invite with a different email from the one their Tailscale account uses. Re-share to the right address. |
| Connection works but is slow | Eduroam is blocking direct UDP, so Tailscale is using a relay. `tailscale ping rpi5` shows `via DERP`. Fine for SSH. |
| `Permission denied (publickey)` | Re-run `ssh-copy-id rpi5`. If passwords are already off, ask Adam to add your `~/.ssh/id_ed25519.pub` to `/home/<you>/.ssh/authorized_keys`. |
