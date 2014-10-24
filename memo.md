
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

* 大会は一周すれば予選通過
* 最速は30秒以内
* 重くする方法
    * 単三電池を入れる
    * その上バッテリーも詰める

