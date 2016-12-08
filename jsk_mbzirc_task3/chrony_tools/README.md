####chrony tools

### server and client

## server:

run ./scripts/install_chrony.sh server

after that comment four lines in 
/etc/chrony/chrony.conf

`
 server 0.debian.pool.ntp.org minpoll 3 maxpoll 15 maxdelay .010
 server 1.debian.pool.ntp.org minpoll 3 maxpoll 15 maxdelay .010
 server 2.debian.pool.ntp.org minpoll 3 maxpoll 15 maxdelay .010
 server 3.debian.pool.ntp.org minpoll 3 maxpoll 15 maxdelay .010
`
then restart the chrony service 
`
chronyc tracking
`
and check if it is working normal

## client:

run ./scripts/install_chrony.sh client server_ip

after that edit two numbers in  
/etc/chrony/chrony.conf

`
server jsk-m100-a-v minpoll 0 maxpoll 10 maxdelay .01
`

Then restart chrony service

