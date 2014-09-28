#ifndef _GPSR_h
#define _GPSR_h

/*
  This code is revised to create and detect wormhole attack in end of 2013.
*/
using namespace std;

// GPSR for ns2 w/wireless extensions

#include "agent.h"
#include "ip.h"
#include "delay.h"
#include "scheduler.h"
#include "queue.h"
#include "trace.h"
#include "arp.h"
#include "ll.h"
#include "mac.h"
#include "mac-802_11.h"
#include "priqueue.h"
#include "node.h"
#include "timer-handler.h"
#include <random.h>
#include "god.h"


#include "../locservices/ls_queued_timer.h"


/***********/
/* Defines */
/***********/

// Location Services
#define _OMNI_     0
#define _REACTIVE_ 1
#define _GRID_     2
#define _CELL_     3

// GPSR Defaults - Karp
#define GPSR_ALIVE_DESYNC  0.5	/* desynchronizing term for alive beacons */
#define GPSR_ALIVE_INT     0.5	/* interval between alive beacons */
#define GPSR_ALIVE_EXP     (3*(GPSR_ALIVE_INT+GPSR_ALIVE_DESYNC*GPSR_ALIVE_INT))
				/* timeout for expiring rx'd beacons */
#define GPSR_PPROBE_INT    1.5	/* interval between perimeter probes */
#define GPSR_PPROBE_DESYNC 0.5	/* desynchronizing term for perimeter probes */
#define GPSR_PPROBE_EXP    8.0	/* how often must use a perimeter to keep probing them */
#define GPSR_PPROBE_RTX    1

#define PERI_DEFAULT_HOPS 32	/* default max number of hops in peri header */
#define MAX_PERI_HOPS_STATIC 128
#define PLANARIZE_RNG 0
#define PLANARIZE_GABRIEL 1

// SendBuffer
#define BUFFER_CHECK    0.05
#define SEND_BUF_SIZE   64   /* As in DSR */
#define SEND_TIMEOUT    30   /* As in DSR */
#define JITTER_VAR      0.3

// Reactive Beaconing
#define GPSR_RBEACON_JITTER   0.015
#define GPSR_RBEACON_RETRIES  1
#define GPSR_BEACON_REQ_DELAY bexp_
#define GPSR_BEACON_DELAY     bexp_

// Packet Types
#define GPSR_PKT_TYPES    6     /* how many pkt types are defined */

#define GPSRH_DATA_GREEDY 0	/* greedy mode data packet */
#define GPSRH_DATA_PERI   1	/* perimeter mode data packet */
#define GPSRH_PPROBE      2	/* perimeter probe packet */
#define GPSRH_BEACON      3     /* liveness beacon packet */
#define GPSRH_BEACON_REQ  4     /* neighbor request */

#define GPSR_ROUTE_VERBOSE 1   /* should shortest route be aquired and */


#ifndef CURRTIME
#define CURRTIME Scheduler::instance().clock()
#endif


// opaque type: returned by NeighbTable iterator, holds place in table
typedef unsigned int NeighbTableIter;


class GPSR_Agent;
class NeighbEnt;
class NeighbTable;


/**************/
/* Structures */
/**************/

struct GPSRSendBufEntry{
    double t;
    Packet *p;
};

struct PeriEnt {
    double x;
    double y;
    double z;
    nsaddr_t ip;
};

struct hdr_gpsr {

    struct PeriEnt hops_[MAX_PERI_HOPS_STATIC];
    struct PeriEnt peript_; // starting point
    struct PeriEnt perips_; // intersection point
    nsaddr_t periptip_[3];
    int nhops_;
    int currhop_;
    int mode_;

    // Additions
    int load;
    int retry;

    // for geo-anycast - wk
     bool geoanycast;



    enum port_t { GPSR=0, LOCS=1 };
    int port_;

    int size() { return 0; }

    // NS-2 requirements
    static int offset_;
    inline static int& offset() { return offset_; }
    inline static hdr_gpsr* access(const Packet* p) {
	return (hdr_gpsr*) p->access(offset_);
    }

    void add_hop(nsaddr_t addip, double addx, double addy, double addz) {
	if (nhops_ == MAX_PERI_HOPS_STATIC) {
	    fprintf(stderr, "hdr_gpsr::add_hop: out of slots!\n");
	    abort();
	}
	hops_[nhops_].x = addx; hops_[nhops_].y = addy; hops_[nhops_].z = addz;
	hops_[nhops_].ip = addip;
	nhops_++;
    }
};


/*****************/
/* Timer Classes */
/*****************/

class PacketDelayTimer : public QueuedTimer {

 public:
    PacketDelayTimer(GPSR_Agent *a_, int size) : QueuedTimer(size)
	{ a = a_; }
    void handle();
    void deleteInfo(void* info);

 private:
    GPSR_Agent *a;

};

class GPSRSendBufferTimer : public TimerHandler {

 public:
    GPSRSendBufferTimer(GPSR_Agent *a): TimerHandler() { a_ = a; }
    void expire(Event *e);

 protected:
    GPSR_Agent *a_;

};

class GPSR_BeaconTimer : public TimerHandler {

 public:
    GPSR_BeaconTimer(GPSR_Agent *a_) { a = a_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
};

class GPSR_LastPeriTimer : public TimerHandler {

 public:
    GPSR_LastPeriTimer(GPSR_Agent *a_) { a = a_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
};

class GPSR_PlanarTimer : public TimerHandler {

 public:
    GPSR_PlanarTimer(GPSR_Agent *a_) { a = a_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
};


class GPSR_DeadNeighbTimer : public TimerHandler {

 public:
    GPSR_DeadNeighbTimer(GPSR_Agent *a_, NeighbEnt *ne_)
	{ a = a_; ne = ne_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
    NeighbEnt *ne;
};

class GPSR_PeriProbeTimer : public TimerHandler {

 public:
    GPSR_PeriProbeTimer(GPSR_Agent *a_, NeighbEnt *ne_)
	{ a = a_; ne = ne_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
    NeighbEnt *ne;
};

class GPSRBeaconDelayTimer : public TimerHandler {

 public:
    GPSRBeaconDelayTimer(GPSR_Agent *a_) { a = a_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
};

class GPSRBeaconReqDelayTimer : public TimerHandler {

 public:
    GPSRBeaconReqDelayTimer(GPSR_Agent *a_) { a = a_; }
    virtual void expire(Event *);

 protected:
    GPSR_Agent *a;
};

/******************/
/* Neighbor Entry */
/******************/

class NeighbEnt {

 public:
     NeighbEnt(GPSR_Agent *ina) :
	 peri(NULL), perilen(0), maxlen(0), dnt(ina, this), ppt(ina, this)
       {
       };

     void planarize(class NeighbTable *, int, double, double, double); /** [HMF] Screen this edge */

     int closer_pt(nsaddr_t myip, double myx, double myy, double myz,
		   double ptx, double pty, nsaddr_t ptipa, nsaddr_t ptipb,
		   double dstx, double dsty, double *closerx, double *closery);


     nsaddr_t dst;	   //**< [HMF] IP of neighbor
     double x, y, z;       //**< [HMF] location of neighbor last heard
     double ts;            //**< [HMF] timestamp of location information
     struct PeriEnt *peri; //**< [HMF] Perimeter via this neighbor
     int perilen;	   //**< [HMF] length of perimeter
     int maxlen;	   //**< [HMF] allocated slots in peri
     int live;	           //**< [HMF] when planarizing, whether edge should be used
     int load;	           //**< [MT] Load on MAC layer (802.11) at this neighbor (= 0..100)
     GPSR_DeadNeighbTimer dnt; //**< [HMF] timer for expiration of neighbor
     GPSR_PeriProbeTimer ppt;  //**< [HMF] Timer for generation of perimeter probe to neighbour
     int public_key_;              // the public key for each entry
     int key_ [10];                // the number of private key for each entry
};

/******************/
/* Neighbor Table */
/******************/
/** Array that is ordered by destination addr and holds the NeighbEnts */

class NeighbTable {

 public:
     NeighbTable(GPSR_Agent *mya);
     ~NeighbTable();

     void ent_delete(const NeighbEnt *ent);          //** Delete an entry
     void planarize(int, int, double, double, double);          //** Remove all crossing edges

     inline double norm(double tmp_bear){
       double to_norm = tmp_bear;
       while(to_norm <= 0)
	 to_norm += 2*M_PI;
       return to_norm;
     }

     inline double norm_rev(double tmp_bear){
       double to_norm = tmp_bear;
       while(to_norm >= 0)
	 to_norm -= 2*M_PI;
       return to_norm;
     }

     NeighbEnt *ent_add(const NeighbEnt *ent); //** [HMF] Add an entry
     NeighbEnt *ent_finddst(nsaddr_t dst);           //** [HMF] Find an entry by his destination address

     // Neighbor Functions
     class NeighbEnt *ent_findshortest       //** Find Closest
	 (MobileNode *mn, double x, double y, double z);
     class NeighbEnt *ent_findshortest_cc    //** Find Closest with congestion control
	 (MobileNode *mn, double x, double y, double z, double alpha);
     class NeighbEnt *ent_findshortestXcptLH //** Find Closest that is not the LastHop
	 (MobileNode *mn, nsaddr_t lastHopId, double x, double y, double z);

     //** [HMF] Iterating through every table on peri and return the first
     //*  hop on the perimeter that is close to the destination than
     //*  itself
     class NeighbEnt *ent_findcloser_onperi
	 (MobileNode *mn, double x, double y, double z, int *perihop);
     class NeighbEnt *ent_findcloser_edgept
	 (MobileNode *, double, double, nsaddr_t, nsaddr_t, double, double, double *, double *);
     class NeighbEnt *ent_next_ccw(MobileNode *, NeighbEnt *, int);
     class NeighbEnt *ent_next_ccw(double, double, double, int, NeighbEnt * = 0);
     class NeighbEnt *ent_findface(MobileNode *, double, double, double, int);
     NeighbTableIter InitLoop();
     class NeighbEnt *NextLoop(NeighbTableIter *);
     class NeighbEnt *ent_findnext_onperi(MobileNode *, int, double, double, double, int);
     class NeighbEnt *ent_findnextcloser_onperi(MobileNode *mn, double dx, double dy, double dz);
     int meanLoad();         //**< calculates the mean load of all neighbors in this table
     inline int noEntries() {return nents;}

     /**set for planarization, entries are valid edges*/
     bool counter_clock;

     DHeapEntry *val_item;
     DHeap *valid;
     int itedge;
 protected:
  friend class NeighbEnt;

 private:

     int nents;		     //** Entries currently in use
     int maxents;
     GPSR_Agent *a;
     NeighbEnt **tab;
};

/**************/
/* GPSR Agent */
/**************/

class GPSR_Agent : public Tap, public Agent {

 public:
    GPSR_Agent(void);

    // Timer called Functions
    void beacon_callback(void);	                   // generate a beacon (timer-triggered)
    void deadneighb_callback(class NeighbEnt *ne); // neighbor gone (timer/MAC-trig)
    void periprobe_callback(class NeighbEnt *ne);  // gen perimeter probe (timer-trig)
    void lastperi_callback(void);	           // turn off peri probes when unused for timeout
    void planar_callback(void);	                   // planarization callback
#ifdef SPAN
    void span_callback(void);                      // SPAN callback
#endif

    virtual int command(int argc, const char * const * argv);
    void lost_link(Packet *p);
    void tap(const Packet *p);

    // Additions
    const inline int isActive() { return(active_); }
    void sleep();  /** [HMF] This functions lays the Agent to rest */
    void wake();   /** [HMF] This functions wakes the Agent up and reinits its times  */

    void allow(unsigned int pktType) { send_allowed[pktType] = true; }
    void block(unsigned int pktType) { send_allowed[pktType] = false; }
    bool allowedToSend(unsigned int pktType) { return (send_allowed[pktType]); }

 protected:

    bool send_allowed[GPSR_PKT_TYPES]; //**< array with permission value to send pkttype

    bool     malicious;           // malicious node

    int off_gpsr_;		 //**< offset of the GPSR packet header in pkt
    int use_mac_;		 //**< whether or not to simulate full MAC level
    int use_peri_;		 //**< whether or not to use perimeters
    int verbose_;		 //**< verbosity (binary)
    int active_;                 //**< specifies if node is active [HMF]
    int drop_debug_;		 //**< whether or not to be verbose on NRTE events
    int peri_proact_;		 //**< whether or not to pro-actively send pprobes
    int use_implicit_beacon_;	 //**< whether or not all data packetsare beacons
    int use_planar_;		 //**< whether or not to planarize graph
    int use_loop_detect_;	 //**< whether or not to fix loops in peridata pkts
    int use_timed_plnrz_;	 //**< whether or not to replanarize w/timer
    int use_beacon_;             //**< whether or not to do beacons at all [MK]
    int use_congestion_control_; //**< whether or not to ship load information with beacons [MT]
    int use_reactive_beacon_;    //**< whether or not to use reactive beaconing [MK]
    int locservice_type_;        //**< which Location Service should be used [MK]
    double bint_;		 //**< beacon interval
    double bdesync_;		 //**< beacon desync random component range
    double bexp_;		 //**< beacon expiration interval
    double pint_;		 //**< perimeter probe interval
    double pdesync_;		 //**< perimeter probe desync random cpt. range
    double lpexp_;		 //**< perimeter probe generation timeout
    double cc_alpha_;		 //**< parameter for congestion control [MT]
    int use_span_;               //**< whether or not to use span services [CL]
    int packet_number_;          //number of all packets
    int trust_packet_number_;       // number of trust packet
    int malicious_packet_number_;   //number of malicious packet

    friend class NeighbEnt;
    class MobileNode *mn_;	        //**< MobileNode
    class PriQueue *ifq_;	        //**< InterfaceQueue [MK]
    class Mac *m;                       //**< MAC
    class Trace *tracetarget;		//**< Trace Target
    class LocationService *locservice_; //**< LocationService [MK]
    class NeighbTable *ntab_;           //**< Neighbor Table

    class GPSR_BeaconTimer   *beacon_timer_;   //**< Alive Beacon Timer
    class GPSR_LastPeriTimer *lastperi_timer_; //**< Last Perimeter Used Timer
    class GPSR_PlanarTimer   *planar_timer_;   //**< Inter-Planarization Timer
    class PacketDelayTimer   *pd_timer;        //**< Packet Delay Timer [MK]

    GPSRSendBufferTimer send_buf_timer;
    GPSRSendBufEntry send_buf[SEND_BUF_SIZE];

    // Reactive Beaconing
    class GPSRBeaconDelayTimer    *beacon_delay_;    //**< Min Delay between two Beacons
    class GPSRBeaconReqDelayTimer *beaconreq_delay_; //**< Min Delay between two Beacon Reqs

    friend class PacketDelayTimer;
    friend class GPSRSendBufferTimer;

    friend class NeighbTable;

    virtual void recv(Packet *, Handler *);
    void trace(char *fmt, ...);
    void tracepkt(Packet *, double, int, const char *);
    void init();

    void forwardPacket(Packet *, int = 0);      //**< Forwarding Packets (Way too big for one function :( )
    void periIn(Packet *, hdr_gpsr *, int = 0);
    int hdr_size(Packet* p);                    //**< [MK] Handles everything size related
    int crosses(class NeighbEnt *, hdr_gpsr *);
    int getLoad();				//**< [MT] recalculate my own load (using neighbors')

    // Beaconing Functions
    void beacon_proc(int, double, double, double, int = -1);
    void recvBeacon(Packet*);                   //**< [MK] receive and evaluate Beacon
    void recvBeaconReq(Packet*);                //**< [MK] receive Beacon Request
    void sendBeacon(double = 0.0);        //**< [MK] send Beacon
    void sendBeaconRequest();                   //**< [MK] send Beacon request
    void checkGreedyCondition(const Packet*);   //**< [MK] check if node is a Greedy Neighbor to src

    // SendBuffer Functions
    void notifyPos(nsaddr_t);
    void stickPacketInSendBuffer(Packet *p);
    void dropSendBuff(Packet *& p, const char*);
    void sendBufferCheck();
    void Terminate();
    double sendbuf_interval() {
      return BUFFER_CHECK;
    }

};

#endif //_GPSR_h

