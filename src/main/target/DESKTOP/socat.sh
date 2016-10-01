USERNAME=ntsoi sudo socat -d -d -d -d -lf /tmp/socat pty,link=/dev/fc_serial,raw,echo=0,user=${USERNAME},group=staff pty,link=/dev/host_serial,raw,echo=0,user=${USERNAME},group=staff
