======================================================================
# Define Protocol Defaults
# ======================================================================

## GPSR Options
Agent/GPSR set bdesync_                0.5 ;# beacon desync random component
Agent/GPSR set bexp_                   [expr 3*([Agent/GPSR set bint_]+[Agent/GPSR set bdesync_]*[Agent/GPSR set bint_])] ;# beacon timeout interval
Agent/GPSR set pint_                   1.5 ;# peri probe interval
Agent/GPSR set pdesync_                0.5 ;# peri probe desync random component
Agent/GPSR set lpexp_                  8.0 ;# peris unused timeout interval
Agent/GPSR set drop_debug_             1   ;#
Agent/GPSR set peri_proact_            1 	 ;# proactively generate peri probes
Agent/GPSR set use_implicit_beacon_    1   ;# all packets act as beacons; promisc.
Agent/GPSR set use_timed_plnrz_        0   ;# replanarize periodically
Agent/GPSR set use_congestion_control_ 0
Agent/GPSR set use_reactive_beacon_    0   ;# only use reactive beaconing

set val(bint)           0.5  ;# beacon interval
set val(use_mac)        1    ;# use link breakage feedback from MAC
set val(use_peri)       1    ;# probe and use perimeters
set val(use_planar)     1    ;# planarize graph
set val(verbose)        1    ;#
set val(use_beacon)     1    ;# use beacons at all
set val(use_reactive)   0    ;# use reactive beaconing
set val(locs)           0    ;# default to OmniLS
set val(use_loop)       0    ;# look for unexpected loops in peris

set val(agg_mac)          1 ;# Aggregate MAC Traces
set val(agg_rtr)          0 ;# Aggregate RTR Traces
set val(agg_trc)          0 ;# Shorten Trace File

# ======================================================================
# Define NS Object Defaults
# ======================================================================

# In case normal MovementTrace Option is a no-go
Node/MobileNode set movtrace_   1

# Should ARP Lookup be used in LL
LL set useARP_                  0

# Routing Protocol Prefered (might break Protocol)
Queue/DropTail/PriQueue set Prefer_Routing_Protocols  0

# ======================================================================
# Define Options
# ======================================================================

set val(chan)		Channel/WirelessChannel
set val(prop)		Propagation/TwoRayGround
set val(netif)		Phy/WirelessPhy
set val(mac)		Mac/802_11
set val(ifq)		Queue/DropTail/PriQueue
set val(ll)		LL
set val(ant)		Antenna/OmniAntenna
set val(x)		1000      ;# X dimension of the topography
set val(y)		1000      ;# Y dimension of the topography
set val(ifqlen)		512       ;# max packet in ifq
set val(seed)		1.0
set val(adhocRouting)	GPSR      ;# AdHoc Routing Protocol
set val(nn)		15       ;# how many nodes are simulated
set val(stop)		40.0     ;# simulation time
set val(use_gk)		0	  ;# > 0: use GridKeeper with this radius
set val(zip)		0         ;# should trace files be zipped
set val(bw)		""
set val(bs)		""
set val(rr)		""

set path                ./
set val(cp)             ""
set val(sc)             ""
set val(out)            ""
set val(on_off)		""
set val(pingLog)        ""
set val(lt)		"" ;# MAC load trace file

set val(cc)		"" ;# congestion control
set val(smooth_cc)	""

set val(agttrc)         ON ;# Trace Agent
set val(rtrtrc)         ON ;# Trace Routing Agent
set val(mactrc)         ON ;# Trace MAC Layer
set val(movtrc)         ON ;# Trace Movement

set val(mac_trace)      "" ;# dummy

set val(ed)             " "
set val(ve)             " "
set val(energymodel)   EnergyModel		    ;# Energy Model
set val(initialenergy) 100			    ;# value

#Mac/802_11 set RTSThreshold_  3000
#Mac/802_11 set basicRate_ 1Mb 
#Mac/802_11 set dataRate_  2Mb

# =====================================================================
# User defined Procedures
# ======================================================================
proc usage {} {
    global argv0
    puts "\nUsage: ns $argv0 -out tracefile\n"
    puts "    NS Options:"
    puts "     -nn           \[number of nodes\]"
    puts "     -stop         \[simulation duration in secs\]"
    puts "     -x / -y       \[dimension in meters\]"
    puts "     -adhocRouting \[routing protocol to use\]"
    puts "     -use_gk       \[radius for gridkeeper usage\]"
    puts "     -zip          \[(0/1) should tracefiles be zipped on-the-fly\]"
    puts "     -cc           \[alpha for congestion control ((MAC802_11 only)\]"
    puts "     -ifqlen       \[max packets in interface queue\]"
    puts ""
    puts "    File Options:"
    puts "     -cp      \[traffic pattern\]"
    puts "     -sc      \[scenario file\]"
    puts "     -on_off  \[wake/sleep pattern\]"
    puts "     -lt      \[load trace file (MAC802_11 only)\]"
    puts "     -pingLog \[log file for ping statistics (Ping Traffic only)\]"
    puts ""
    puts "    MAC Options:"
    puts "     -rr           \[radio range in meters\]"
    puts "     -bw           \[link/dataRate bandwidth in bits/sec\]"
    puts "     -bs           \[basicRate bandwidth in bits/sec\]"
    puts ""
    puts "    GPSR Options:"
    puts "     -bint         \[beacon interval (and beacon expiry)\]"
    puts "     -use_planar   \[(0/1) planarize graph\]"
    puts "     -use_peri     \[(0/1) use perimeter mode\]"
    puts "     -use_mac      \[(0/1) use mac callback\]"
    puts "     -verbose      \[(0/1) be verbose\]"
    puts "     -use_beacon   \[(0/1) use beacons at all (disable beacons with 0)\]"
    puts "     -use_reactive \[(0/1) use reactive beaconing\]"
    puts "     -locs         \[locservice to use (0-Omni/1-RLS/2-GLS/3-HLS)\]"
    puts "     -use_loop     \[(0/1) use loop detection\]"
    puts ""
    puts "     -ed           \[topology file (edges)\]"
    puts "     -ve           \[topology file (verteces)\]"
    puts ""
}

proc getopt {argc argv} {
    global val
    lappend optlist cp sc on_off om out pingLog nn stop x y adhocRouting mac_emu rr bw lt use_gk ifqlen ora
    # HGPS
    lappend optlist upd bint cval mgrid tper tqo
    # GPSR
    lappend optlist bint use_planar use_peri use_mac verbose use_beacon cc smooth_cc use_reactive use_loop
    # CBF & LOCS
    lappend optlist locs supt use_rec pkt_ret rev_ord use_la use_lazy use_uctf 
    lappend use_randa use_sdd agg_mac agg_rtr agg_trc
    # GSR
    lappend optlist ed ve
    lappend optlist seed mac_trace zip no_echo

    for {set i 0} {$i < $argc} {incr i} {
	set arg [lindex $argv $i]
	if {[string range $arg 0 0] != "-"} continue
	set name [string range $arg 1 end]
	set val($name) [lindex $argv [expr $i+1]]
    }
    if { $val(out) == "" } {
	usage
	exit
    }
}

proc printparams {} {
    global val  
    puts "\nParameterset:"
    puts "Tracefile: \"$val(out)\""
    puts "Protocol: $val(adhocRouting) nn: $val(nn) stop: $val(stop) x: $val(x) y: $val(y)"
    puts "Radio Range: $val(rr)"

    if { ($val(adhocRouting) == "GPSR") } {
	if { $val(locs) == "0" } {
	    puts "$val(adhocRouting)/OMNI: Omnipotent Location Service selected."
	} elseif { $val(locs) == "1" } {
	    puts "$val(adhocRouting)/RLS: Reactive Location Service selected."
	} elseif { $val(locs) == "2" } {
	    puts "$val(adhocRouting)/GLS: Grid Location Service selected."
	} elseif { $val(locs) == "3" } {
	    puts "$val(adhocRouting)/HLS: Cell Location Service selected."
	} else {
	    puts "$val(adhocRouting)/UKN: Unknown Location Service. Defaulting to Omnipotent Location Service."
	}
    }
    if { $val(cc) != "" } {
	puts "Using congestion control with alpha = $val(cc) ..."
    }
    puts ""
}

proc changeActiveState {nId on} {
    global node_ val
    if {$on == 0} {
	#puts "Turning off node $nId"
	if { ($val(adhocRouting) == "DSR")||($val(adhocRouting) == "GPSR")||($val(adhocRouting) == "AODV") } {
	    set r [$node_($nId) set ragent_]
	    $r sleep
	}
    } else {
	#puts "Turning on node $nId"
	if { ($val(adhocRouting) == "DSR")||($val(adhocRouting) == "GPSR")||($val(adhocRouting) == "AODV") } {
	    set r [$node_($nId) set ragent_]
	    $r wake
	}
    }
}

proc turnoff {nId} {
    global node_ val

	#puts "Turning off node $nId"
	set r [$node_($nId) set ragent_]
	 $r sleep
}

proc estimEnd {startTime simTime simEndTime} {
    set now         [clock seconds]
    set realGone    [expr $now - $startTime]
    set simToGo     [expr $simEndTime - $simTime]
    set percSimGone [expr ($simTime / $simEndTime) * 100]
    set percSimToGo [expr 100 - $percSimGone]
    if {$percSimGone == 0} {
	set ete 0
	set eteString "unknown"
    } else {
	set ete [expr $startTime + ($realGone / $percSimGone) * 100]
    }
    set eteString   [clock format [expr round($ete)]]
    set sTimeString [clock format $startTime]
    if {$ete != 0} {
	puts "$simTime\tRun: $realGone ETE:\t$eteString"
    } else {
	puts "$simTime\tBeginn: $sTimeString!"
    }
}

proc instEstim {startTime simEndTime step} {
    global ns_
    for {set t 1} {$t < $simEndTime } { set t [expr  $t + $step]} {
	$ns_ at $t "estimEnd $startTime $t $simEndTime"
    }
}

proc create_gridkeeper {} {
    global gkeeper val node_
 
    set gkeeper [new GridKeeper]
 
    puts "Initializing GridKeeper with radius $val(use_gk) ..."
    #initialize the gridkeeper
 
    $gkeeper dimension $val(x) $val(y)
 
    #
    # add mobile node into the gridkeeper, must be added after
    # scenario file
    #

    for {set i 0} {$i < $val(nn) } {incr i} {
        $gkeeper addnode $node_($i)
 
        $node_($i) radius $val(use_gk)
    }
   # $node_(9)color red
 
}

# =====================================================================
# Main Program
# ======================================================================
getopt $argc $argv

if { $val(adhocRouting) == "GPSR" } {
    Agent/GPSR set locservice_type_ $val(locs)
}

# create trace object for ping
if { $val(pingLog) != "" } {
    set pingLog  [open $val(pingLog) w] 
} else {
    set pingLog  $val(pingLog)
}

# create trace object for MAC load
if { $val(mac) == "Mac/802_11" } {
    if { $val(lt) != "" } {
	set loadTrace  [open $val(lt) w]
        puts $loadTrace "# x=$val(x), y=$val(y), n=$val(nn), stop=$val(stop)"
    } else {
	set loadTrace  $val(lt)
    }
}

# set up MAC load scanning
if { $val(cc) != "" || $val(lt) != "" } {
    Mac/802_11 set scan_int_	0.001	;# scanning interval
    Mac/802_11 set scan_len_	200	;# scan count for each probe
    if { $val(smooth_cc) == "1" } {
	Mac/802_11 set smooth_scan_ 1	;# smooth the scanned values
    }
}

# set up congestion control
if { $val(cc) != "" } {
    Agent/GPSR set use_congestion_control_ 1
    Agent/GPSR set cc_alpha_ $val(cc)
} else {
    Agent/GPSR set cc_alpha_ 0
}

# set up headers as needed to save on memory
add-all-packet-headers
remove-all-packet-headers
add-packet-header Common Flags IP LL Mac Message GPSR  LOCS SR RTP Ping HLS
# PKT Types of special Interest: 
# ARP TCP GPSR LOCS HGPS SR DSDV AODV TORA IMEP Message Ping RTP 
puts "\n !Warning! Don't forget to check header-inclusion "
puts "           (Not needed for GPSR/DSR & CBR/Ping)\n"


# =====================================================================
# Other default settings

set AgentTrace			ON
set RouterTrace			ON
set MacTrace			OFF

LL set mindelay_		50us
LL set delay_			25us
LL set bandwidth_		0	;# not used
LL set off_prune_		0	;# not used
LL set off_CtrMcast_		0	;# not used

Agent/Null set sport_		0
Agent/Null set dport_		0

Agent/CBR set sport_		0
Agent/CBR set dport_		0

Agent/TCPSink set sport_	0
Agent/TCPSink set dport_	0

Agent/TCP set sport_		0
Agent/TCP set dport_		0
Agent/TCP set packetSize_	1460

Queue/DropTail/PriQueue set Prefer_Routing_Protocols    0

# unity gain, omni-directional antennas
# set up the antennas to be centered in the node and 1.5 meters above it
Antenna/OmniAntenna set X_ 0
Antenna/OmniAntenna set Y_ 0
Antenna/OmniAntenna set Z_ 1.5
Antenna/OmniAntenna set Gt_ 1.0
Antenna/OmniAntenna set Gr_ 1.0

# Initialize the SharedMedia interface with parameters to make
# it work like the 914MHz Lucent WaveLAN DSSS radio interface
Phy/WirelessPhy set CPThresh_ 10.0
Phy/WirelessPhy set CSThresh_ 1.559e-11
Phy/WirelessPhy set RXThresh_ 3.652e-10
Phy/WirelessPhy set Rb_ 2*1e6
#Phy/WirelessPhy set CSThresh_ 2.78831e-9    ;#100m
#Phy/WirelessPhy set RXThresh_ 1.11532e-8    ;#50m
#Phy/WirelessPhy set CSThresh_ 6.97078e-10   ;#200m
#Phy/WirelessPhy set RXThresh_ 2.78831e-9    ;#100m
Phy/WirelessPhy set Pt_ 0.2818
Phy/WirelessPhy set freq_ 914e+6 
Phy/WirelessPhy set L_ 1.0

#Agent/GPSR set verbose_ 1
Agent/GPSR set use_mac_ 1
Agent/GPSR set use_peri_ 1
#Agent/GPSR set drop_debug_ 1
Agent/GPSR set peri_proact_ 0
Agent/GPSR set use_implicit_beacon_ 1
Agent/GPSR set use_planar_ 1
# =====================================================================
Agent/GPSR set bint_                  $val(bint)
# Recalculating bexp_ here
Agent/GPSR set bexp_                 [expr 3*([Agent/GPSR set bint_]+[Agent/GPSR set bdesync_]*[Agent/GPSR set bint_])] ;# beacon timeout interval
Agent/GPSR set use_peri_              $val(use_peri)
Agent/GPSR set use_planar_            $val(use_planar)
Agent/GPSR set use_mac_               $val(use_mac)
Agent/GPSR set use_beacon_            $val(use_beacon)
Agent/GPSR set verbose_               $val(verbose)
Agent/GPSR set use_reactive_beacon_   $val(use_reactive)
Agent/GPSR set use_loop_detect_       $val(use_loop)

CMUTrace set aggregate_mac_           $val(agg_mac)
CMUTrace set aggregate_rtr_           $val(agg_rtr)
God set shorten_trace_                $val(agg_trc)

if { $val(movtrc) == "OFF" || $val(agg_trc) == 1} {
Node/MobileNode set movtrace_ 0
}

# seeding RNG
ns-random $val(seed)

# set MACTRACE option
if { $val(mac_trace) != "" } {
    set val(mactrc)         $val(mac_trace)
    puts "MAC trace is $val(mactrc)"
}
#=====================================================================
# Initialize trace file desctiptors
#=====================================================================
# *** Throughput Trace ***
set f0 [open out02.tr w]
set f1 [open out12.tr w]
set f2 [open out22.tr w]
set f3 [open out32.tr w]

# *** Packet Loss Trace ***
set f4 [open lost02.tr w]
set f5 [open lost12.tr w]
set f6 [open lost22.tr w]
set f7 [open lost32.tr w]

# *** Packet Delay Trace ***
set f8 [open delay02.tr w]
set f9 [open delay12.tr w]
set f10 [open delay22.tr w]
set f11 [open delay32.tr w]


# create simulator instance
set ns_		[new Simulator]

set nf [open out.nam w]
#set f0 [open out0.tr w]
# for Xgraph
#$ns_ namtrace-all $nf
#proc finish {} {
#global ns_ nf f0
#$ns_ flush-trace
#close $nf
#close $f0
#exec nam out.nam &
#exec xgraph out0.tr -geometry 800x400 &
#exit 0
#}

# setup topography object
set topo	[new Topography]
$topo load_flatgrid $val(x) $val(y)

# create trace object for ns and nam
if { $val(zip) == "1" } {
    set tracefd [open "|gzip -9c > $val(out).gz" w]
} else {
    set tracefd	[open $val(out) w]
}
$ns_ trace-all $tracefd
set namtrace [open out.nam w]           ;# for nam tracing
$ns_ namtrace-all-wireless $namtrace $val(x) $val(y)

# Start of Xgraph
#proc record {} {
#global sink0 f0
#set ns_ [Simulator instance]
#set time 0.5
#set bw0 [$sink0 set bytes_]
#set now [$ns_ now]
#puts $f0 "$now [expr $bw0/$time*8/1000000]"
#$sink0 set bytes_ 0
#$ns_ at [expr $now+$time] "record"
#}


# Initialize Flags
set holdtime 0
set holdseq 0
set holdtime1 0
set holdseq1 0
set holdtime2 0
set holdseq2 0
set holdtime3 0
set holdseq3 0
set holdrate1 0
set holdrate2 0
set holdrate3 0
set holdrate4 0
 

# Function To record Statistcis (Bit Rate, Delay, Drop)
 
proc record {} {
        global sink sink2 sink3 sink4 f0 f1 f2 f3 f4 f5 f6 f7 holdtime holdseq holdtime1 holdseq1 holdtime2 holdseq2 holdtime3 holdseq3 f8 f9 f10 f11 holdrate1 holdrate2 holdrate3 holdrate4
        set ns [Simulator instance]
    set time 0.9 ;#Set Sampling Time to 0.9 Sec
        set bw0 [$sink set bytes_]
     #   set bw1 [$sink2 set bytes_]
     #   set bw2 [$sink3 set bytes_]
     #   set bw3 [$sink4 set bytes_]
        set bw4 [$sink set nlost_]
     #   set bw5 [$sink2 set nlost_]
     #   set bw6 [$sink3 set nlost_]
     #   set bw7 [$sink4 set nlost_]
        set bw8 [$sink set lastPktTime_]
        set bw9 [$sink set npkts_]
     #   set bw10 [$sink2 set lastPktTime_]
     #   set bw11 [$sink2 set npkts_]
     #   set bw12 [$sink3 set lastPktTime_]
     #   set bw13 [$sink3 set npkts_]
     #   set bw14 [$sink4 set lastPktTime_]
     #   set bw15 [$sink4 set npkts_]
    set now [$ns now]

        

        # Record Bit Rate in Trace Files
        puts $f0 "$now [expr (($bw0+$holdrate1)*8)/(2*$time*1000000)]"
     #   puts $f1 "$now [expr (($bw1+$holdrate2)*8)/(2*$time*1000000)]"
     #   puts $f2 "$now [expr (($bw2+$holdrate3)*8)/(2*$time*1000000)]"
     #   puts $f3 "$now [expr (($bw3+$holdrate4)*8)/(2*$time*1000000)]"

        # Record Packet Loss Rate in File
        puts $f4 "$now [expr $bw4/$time]"
      #  puts $f5 "$now [expr $bw5/$time]"
      #  puts $f6 "$now [expr $bw6/$time]"
      #  puts $f7 "$now [expr $bw7/$time]"

        # Record Packet Delay in File
        if { $bw9 > $holdseq } {
                puts $f8 "$now [expr ($bw8 - $holdtime)/($bw9 - $holdseq)]"
        } else {
                puts $f8 "$now [expr ($bw9 - $holdseq)]"
        }

     #   if { $bw11 > $holdseq1 } {
                #puts $f9 "$now [expr ($bw10 - $holdtime1)/($bw11 - $holdseq1)]"
      #  } else {
               # puts $f9 "$now [expr ($bw11 - $holdseq1)]"
       # }

        #if { $bw13 > $holdseq2 } {
               # puts $f10 "$now [expr ($bw12 - $holdtime2)/($bw13 - $holdseq2)]"
       # } else {
               # puts $f10 "$now [expr ($bw13 - $holdseq2)]"
       # }

        #if { $bw15 > $holdseq3 } {
               # puts $f11 "$now [expr ($bw14 - $holdtime3)/($bw15 - $holdseq3)]"
        #} else {
               # puts $f11 "$now [expr ($bw15 - $holdseq3)]"
       # }


        # Reset Variables
        $sink set bytes_ 0
 #       $sink2 set bytes_ 0
 #       $sink3 set bytes_ 0
 #       $sink4 set bytes_ 0
        $sink set nlost_ 0
 #       $sink2 set nlost_ 0
 #       $sink3 set nlost_ 0
 #       $sink4 set nlost_ 0
        set holdtime $bw8
        set holdseq $bw9
        set  holdrate1 $bw0
     #   set  holdrate2 $bw1
     #   set  holdrate3 $bw2
     #   set  holdrate4 $bw3

    $ns at [expr $now+$time] "record"   ;# Schedule Record after $time interval sec
}


# End of XGRAPH



# create channel
set chanl [new $val(chan)]

# Create God
set god_ [create-god $val(nn)]

# Attach Trace to God
set T [new Trace/Generic]
$T attach $tracefd
$T set src_ -5
$god_ tracetarget $T    

#
# Define Nodes
#
puts "Configuring Nodes ($val(nn))"
$ns_ node-config -adhocRouting $val(adhocRouting) \
                 -llType $val(ll) \
                 -macType $val(mac) \
                 -ifqType $val(ifq) \
                 -ifqLen $val(ifqlen) \
                 -antType $val(ant) \
                 -propType $val(prop) \
                 -phyType $val(netif) \
                 -channel $chanl \
		 -topoInstance $topo \
                 -wiredRouting OFF \
		 -mobileIP OFF \
		 -agentTrace $val(agttrc) \
                 -routerTrace $val(rtrtrc) \
                 -macTrace $val(mactrc) \
                 -movementTrace $val(movtrc)\
                 -energyModel $val(energymodel) \
                 -initialEnergy $val(initialenergy) \
                 -rxPower 35.28e-3 \
                 -txPower 31.32e-3 \
	         -idlePower 712e-6 \
	         -sleepPower 144e-9                  

#
#  Create the specified number of nodes [$val(nn)] and "attach" them
#  to the channel. 
$ns_ color 1 blue
$ns_ color 2 red
$ns_ color 3 green
$ns_ color 4 yellow
for {set i 0} {$i < $val(nn)-3 } {incr i} {
    set node_($i) [$ns_ node]
    $node_($i) random-motion 0		;# disable random motion

    if { $val(adhocRouting) == "GPSR" || $val(adhocRouting) == "GSR" || $val(adhocRouting) == "CBF" || $val(adhocRouting) == "AODV" } {
	set ragent [$node_($i) set ragent_]
	$ragent install-tap [$node_($i) set mac_(0)]
    }

    if { $val(mac) == "Mac/802_11" } {      
	# bind MAC load trace file
	[$node_($i) set mac_(0)] load-trace $loadTrace
    }

    # Bring Nodes to God's Attention
    $god_ new_node $node_($i)
}

# =====================================================================
# Other default settings

# unity gain, omni-directional antennas
# set up the antennas to be centered in the node and 1.5 meters above it
Antenna/OmniAntenna set X_ 0
Antenna/OmniAntenna set Y_ 0
Antenna/OmniAntenna set Z_ 1.5
Antenna/OmniAntenna set Gt_ 1.0
Antenna/OmniAntenna set Gr_ 1.0

# Initialize the SharedMedia interface with parameters to make
# it work like the 914MHz Lucent WaveLAN DSSS radio interface
Phy/WirelessPhy set CPThresh_ 10.0
Phy/WirelessPhy set CSThresh_ 1.559e-12
Phy/WirelessPhy set RXThresh_ 3.652e-13
Phy/WirelessPhy set Rb_ 2*1e6
#Phy/WirelessPhy set CSThresh_ 2.78831e-9    ;#100m
#Phy/WirelessPhy set RXThresh_ 1.11532e-8    ;#50m
#Phy/WirelessPhy set CSThresh_ 6.97078e-10   ;#200m
#Phy/WirelessPhy set RXThresh_ 2.78831e-9    ;#100m
Phy/WirelessPhy set Pt_ 0.2818
Phy/WirelessPhy set freq_ 914e+6 
Phy/WirelessPhy set L_ 1.0

#Agent/GPSR set verbose_ 1
Agent/GPSR set use_mac_ 1
Agent/GPSR set use_peri_ 1
#Agent/GPSR set drop_debug_ 1
Agent/GPSR set peri_proact_ 0
Agent/GPSR set use_implicit_beacon_ 1
Agent/GPSR set use_planar_ 1


 set node_(6) [$ns_ node]
 $node_(6) random-motion 0
 $node_(6) set X_ 200.734144874268
 $node_(6) set Y_ 50.323395671633
 $node_(6) set Z_ 0.000000000000

    if { $val(adhocRouting) == "GPSR" || $val(adhocRouting) == "GSR" || $val(adhocRouting) == "CBF" || $val(adhocRouting) == "AODV" } {
	set ragent [$node_(6) set ragent_]
	$ragent install-tap [$node_(6) set mac_(0)]
    }

    if { $val(mac) == "Mac/802_11" } {      
	# bind MAC load trace file
	[$node_(6) set mac_(0)] load-trace $loadTrace
    }

    # Bring Nodes to God's Attention
    $god_ new_node $node_(6)
 set node_(7) [$ns_ node]
 $node_(7) random-motion 0
 $node_(7) set X_ 700.734144874268
 $node_(7) set Y_ 150.323395671633
 $node_(7) set Z_ 0.000000000000

    if { $val(adhocRouting) == "GPSR" || $val(adhocRouting) == "GSR" || $val(adhocRouting) == "CBF" || $val(adhocRouting) == "AODV" } {
	set ragent [$node_(7) set ragent_]
	$ragent install-tap [$node_(7) set mac_(0)]
    }

    if { $val(mac) == "Mac/802_11" } {      
	# bind MAC load trace file
	[$node_(7) set mac_(0)] load-trace $loadTrace
    }

    # Bring Nodes to God's Attention
    $god_ new_node $node_(7)
   set node_(8) [$ns_ node] 
   $node_(8) set X_ 900.734144874268
   $node_(8) set Y_ 5.323395671633
   $node_(8) set Z_ 0.000000000000
   $node_(8) random-motion 0		;# disable random motion
   if { $val(adhocRouting) == "GPSR" || $val(adhocRouting) == "GSR" || $val(adhocRouting) == "CBF" || $val(adhocRouting) == "AODV" } {
	set ragent [$node_(8) set ragent_]
	$ragent install-tap [$node_(8) set mac_(0)]
    }

    if { $val(mac) == "Mac/802_11" } {      
	# bind MAC load trace file
	[$node_(8) set mac_(0)] load-trace $loadTrace
    }

    # Bring Nodes to God's Attention
    $god_ new_node $node_(8)
# ====================================================================
#set sink0 [new Agent/LossMonitor]
#$ns_ attach-agent $node_(9) $sink0
# 
# Define node movement model
#
puts "Loading scenario file ($val(sc))..."
if {$val(sc) == ""} {
    puts "  no scenario file specified"
    exit
} else {
    source $val(sc)
}
#proc attach-expoo-traffic { node sink size burst idle rate } {
        #Get an instance of the simulator
#        set ns_ [Simulator instance]

        #Create a UDP agent and attach it to the node
#        set source [new Agent/UDP]
#        $ns_ attach-agent $node $source

        #Create an Expoo traffic agent and set its configuration parameters
#        set traffic [new Application/Traffic/Exponential]
#        $traffic set packetSize_ $size
#        $traffic set burst_time_ $burst
#        $traffic set idle_time_ $idle
#        $traffic set rate_ $rate

        # Attach traffic source to the traffic generator
#        $traffic attach-agent $source
        #Connect the source and the sink
#        $ns_ connect $source $sink
#        return $traffic
#}


# 
# Define traffic model
#
puts "Loading connection pattern ($val(cp))..."
if {$val(cp) == ""} {
    puts "  no connection pattern specified"
} else {
    source $val(cp)
}

#
# Define inactive pattern 
#
puts "Loading inactive pattern ($val(on_off))..."
if { $val(on_off) == "" } {
    puts "  no inactive pattern specified"
} else {
    source $val(on_off)
}

#
# Tell nodes when the simulation ends
#
for {set i 0} {$i < $val(nn) } {incr i} {
    $ns_ at $val(stop).0 "$node_($i) reset";
}

$ns_ at  $val(stop).0002 "puts \"NS EXITING... $val(out)\" ; $ns_ halt"

# Print Parameterset
printparams

# start GridKeeper
if { $val(use_gk) > 0 } {
    create_gridkeeper
}

set startTime [clock seconds]
puts "Installing Time Estimator ($startTime)!"
instEstim $startTime $val(stop) 2.5

puts $tracefd "M 0.0 nn $val(nn) x $val(x) y $val(y) rp $val(adhocRouting)"
puts $tracefd "M 0.0 sc $val(sc) cp $val(cp) seed $val(seed)"
puts $tracefd "M 0.0 prop $val(prop) ant $val(ant) mac $val(mac)"
puts $tracefd "M 0.0 on_off $val(on_off)"

puts "Starting Simulation..."
# Define node initial position in nam
for {set i 0} {$i < $val(nn)} {incr i} {

        # 20 defines the node size in nam, must adjust it according to your
        # scenario size.
        # The function must be called after mobility model is defined
        $ns_ initial_node_pos $node_($i) 20
}  
	# $ns_ at 0.0 "[$node_(7) set ragent_] hacker"
       # $ns_ at 15.0 "$node_(7) off"  
        #$ns_ at 0.0 "$node_(1) off" 
        #$ns_ at 24.0 "$node_(1) on" 
        #$ns_ at 25.0 "$node_(6) off" 
    
          
# set source [attach-expoo-traffic $node_(0) $sink0 200 2s 1s 100k]

#set source [attach-expoo-traffic $node_(0) $node_(1) 200 2s 1s 100k]
#$ns_ at 5.0 "$source start"
#$ns_ at 30.0 "$source stop"


# Start Recording at Time 0

$ns_ at 0.0 "record"
$ns_ at 1.4 "$app1 start"                 ;# Start transmission at time t = 1.4 Sec
#$ns_ at 10.0 "$app2 start"               ;# Start transmission at time t = 10 Sec
#$ns_ at 20.0 "$app3 start"               ;# Start transmission at time t = 20 Sec
#$ns_ at 30.0 "$app4 start"               ;# Start transmission at time t = 30 Sec
#$ns_ at 0.0 "record"
#$ns_ at 30 "finish"
proc stop {} {
        global ns_ tracefd f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11
        # Close Trace Files
        close $f0  
        close $f1
        close $f2
        close $f3
        close $f4  
        close $f5
        close $f6
        close $f7
        close $f8
        close $f9
        close $f10
        close $f11
        #for nam
        exec nam out.nam &
        # Plot Recorded Statistics
        exec xgraph out02.tr out12.tr out22.tr out32.tr -geometry 800x400 &
        exec xgraph lost02.tr lost12.tr lost22.tr lost32.tr -geometry 800x400 &
        exec xgraph delay02.tr delay12.tr delay22.tr delay32.tr -geometry 800x400 &

        # Reset Trace File
        $ns_ flush-trace
        close $tracefd
        exit 0
}
$ns_ at 48 "stop"
$ns_ run
