## Optionaly: Install Samba

```
sudo apt-get install samba samba-common smbclient
```

add the following entry to /etc/samba/smb.conf

```
[Mowgli]
  path = /home/ubuntu/MowgliRoverROS
  read only = no
  valid users = ubuntu
```

restart samba

```
sudo systemctl restart smbd
```

set a password for the ubuntu user

```
sudo smbpasswd -a ubuntu
```
