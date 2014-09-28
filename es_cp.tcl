#
# nodes: 9, max conn: 4, send rate: 0.100000, seed: 1, active nodes: 9
#

#$ns_ at 0.012128558015 "[$node_(0) set ragent_] test-query 9"
#set tcp0 [new Agent/TCP/Reno]
#$ns_ attach-agent $node_(0) $tcp0
#$tcp0 set window_ 8
#$tcp0 set fid_ 1
#set ftp0 [new Application/FTP]
#$ftp0 attach-agent $tcp0
#$tcp0 set class_ 1
#$ns_ at 0.05 "$ftp0 start"


# Create Constant four Bit Rate Traffic sources

 

set agent1 [new Agent/UDP]             ;# Create UDP Agent
$agent1 set prio_ 0                   ;# Set Its priority to 0
set sink [new Agent/LossMonitor]  ;# Create Loss Monitor Sink in order to be able to trace the number obytes received
$ns_ attach-agent $node_(0) $agent1     ;# Attach Agent to source node
$ns_ attach-agent $node_(8) $sink ;# Attach Agent to sink node
$ns_ connect $agent1 $sink            ;# Connect the nodesset app1 [new Application/Traffic/CBR]  ;# Create Constant Bit Rate application
$app1 set packetSize_ 512               ;# Set Packet Size to 512 bytes
$app1 set rate_ 600Kb                    ;# Set CBR rate to 200 Kbits/sec
$app1 attach-agent $agent1             ;# Attach Application to agent







