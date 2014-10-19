
##work
+ ev3(telnet)
$ touch [file]
-- telnet shell --
busybox udpsvd -vE 0.0.0.0 69 tftpd ./

+ local
$ make
$ tftp [ev3 ip address]
-- tftp shell --
binary
put [file]


+ ev3(telnet)
quit
-- shell --
$ chmod 777 [file]
$ ./[file]
