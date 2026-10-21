# Remote access to the Pi (Tailscale + SSH)

The Pi lives on university eduroam, which blocks incoming connections, so we
reach it over [Tailscale](https://tailscale.com). The Pi is on Adam's tailnet
and **shared** with Jack and Sami, so each person keeps their own Tailscale
account and sees only the Pi, not each other's devices.

Everyone logs in as the Pi's single `pi` user, each with their own SSH key.
When it is all set up, everyone connects the same way:

```bash
ssh rpi5
```

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

### 1.1 Name it `rpi5`

The hostname becomes the Tailscale name, so set it **before** joining the
tailnet:

```bash
sudo hostnamectl set-hostname rpi5
sudo sed -i 's/^127\.0\.1\.1.*/127.0.1.1\trpi5/' /etc/hosts
```

The second command prints `sudo: unable to resolve host rpi5`. That is
expected and harmless: the Pi has its new name but `/etc/hosts` does not yet,
so sudo complains once and then runs the command, which fixes the entry.

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

1. Click the **Wi-Fi icon** in the top-right of the desktop and choose
   **eduroam**.
2. In the dialog that opens, enter the settings from your university's IT
   "eduroam on Linux" page. They vary between universities. A typical setup is:
   - **Authentication:** Protected EAP (PEAP)
   - **CA certificate:** the one your university names. Don't choose "No CA
     certificate is required" unless IT says to: without it, a fake "eduroam"
     hotspot can collect your password.
   - **Inner authentication:** MSCHAPv2
   - **Username:** your full university email, e.g. `you@university.ac.uk`
   - **Password:** your university password
3. Click **Connect** and check that you're online with `ping -c3 tailscale.com`.

#### Make sure it reconnects with nobody there

A Wi-Fi connection made from the desktop can keep its password in the
desktop **keyring** instead of with the system. Then, after a reboot, the Pi
won't join eduroam until someone unlocks the keyring at the screen, and a
headless Pi just stays offline. Check that it's stored with the system:

```bash
nmcli -g 802-1x.password-flags,connection.permissions connection show eduroam
```

You want `0` followed by an empty value (shown as `0:`). If you see anything
else, fix it. Use the connection's name in place of `eduroam` if it's
different (`nmcli connection show` lists them).

```bash
sudo nmcli connection modify eduroam \
  802-1x.password-flags 0 \
  802-1x.password "YOUR_PASSWORD" \
  connection.permissions "" \
  connection.autoconnect yes
```

**The real test:** unplug the monitor and keyboard, run `sudo reboot`, and
`ssh rpi5` from your laptop a minute later. Tailscale starts at boot by itself,
so if that works, the Pi will come back on its own after power cuts.

> ⚠️ The password is now stored in
> `/etc/NetworkManager/system-connections/eduroam.nmconnection`, readable by
> root. See [Trust](#trust).

### 1.5 Note your Pi's full Tailscale name

Collaborators need the Pi's full name, because a shared device's short name
does not resolve in their tailnet. Run:

```bash
tailscale status --json | grep -m1 DNSName
# "DNSName": "rpi5.tail6c0787.ts.net."
```

Ours is **`rpi5.tail6c0787.ts.net`** (drop the trailing dot). That is what you
give Jack and Sami.

---

## 2. Adam's laptop

1. Install Tailscale from <https://tailscale.com/download> and sign in with the
   same account as the Pi.
2. Add this to `~/.ssh/config`:

   ```
   Host rpi5
       HostName rpi5
       User pi
       IdentityFile ~/.ssh/id_ed25519
       IdentitiesOnly yes
   ```

   `HostName rpi5` works because MagicDNS is on by default in your own tailnet.

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

3. When Adam's share invite arrives by email, accept it. `rpi5` then appears in
   `tailscale status`.
4. Add this to `~/.ssh/config`, using the full name Adam gives you:

   ```
   Host rpi5
       HostName rpi5.tail6c0787.ts.net
       User pi
       IdentityFile ~/.ssh/id_ed25519
       IdentitiesOnly yes
   ```

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
   `rpi5` → `…` → **Share…**
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
the Pi. That includes reading the eduroam password from step 1.4, and adding
their own keys or accounts. Only share the Pi with people you'd trust with
that password. Better still, check whether your university offers separate
credentials or a separate network for devices, and use those instead of your
personal login.

## Troubleshooting

| Symptom | Check |
|---|---|
| `Could not resolve hostname rpi5` | Is Tailscale running on your laptop? Try the `100.x.y.z` address from `tailscale status` in place of the name. |
| `Connection closed by 100.x.y.z port 22`, no password prompt | Your agent offered too many keys. Add `IdentitiesOnly yes` to the `rpi5` block, wait a minute, and try again. |
| `rpi5` shows as offline in `tailscale status` | The Pi has no internet. Check eduroam with a keyboard: `nmcli connection show --active`, and the [reconnect check](#make-sure-it-reconnects-with-nobody-there). |
| Collaborator can't see `rpi5` | They accepted the invite with a different email from the one their Tailscale account uses. Re-share to the right address. |
| Connection works but is slow | Eduroam is blocking direct UDP, so Tailscale is using a relay. `tailscale ping rpi5` shows `via DERP`. Fine for SSH. |
| `Permission denied (publickey)` | Your key isn't in `~/.ssh/authorized_keys` on the Pi. Send your `~/.ssh/id_ed25519.pub` to Adam again. |
