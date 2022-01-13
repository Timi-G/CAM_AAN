from math import dist
import random

from matplotlib import pyplot as plt
import numpy as np

import cam_airnavconfrules as ccf


'''Object Classes'''
# dept:departure, dest:destination, size:object_size_class
# pos:current_position, fir_pos:first_position, sec_pos:second_position
# agg_pos:aggregate_positions, t_step:time_step
class Air_Object:

    def __init__(self,dept,dest):
        self.dept = dept
        self.dest = dest
        self.tdes = []
        self.size = 1
        self.con_rad = []
        self.ccrad = 0
        self.way_p = []
        self.pos = dept
        self.agg_pos = []
        self.t_step = 0
        self.distn = 0
        self.agg_distn = []

# to define object path; combining the object's different progressive positions
    def collect_pos(self):
        self.agg_pos=self.agg_pos+self.pos
        if self.agg_pos[-3:-1] != self.dest:
            self.t_step += 1

# the object path as a 2D array
    def flight_path(self):
        fp = np.array(self.agg_pos)
        nfp = fp.reshape(-1,2)
        self.fp = nfp.T

        # convert numpy to horizontal list for better result display
        ntl= fp.tolist()
        self.agg_pos=ccf.conv_to_2d(ntl,2)

# to calculate & collect distance of obj pos to dest
    def collect_distn(self):
        self.distn = dist(self.pos,self.dest)
        self.agg_distn+=[self.distn]

# class for other objects in the air during flight
class Free_Air_Object:

    def __init__(self,size,pos):
        self.pos = pos
        self.size = size
        self.con_rad = []
        self.dens = 0
        self.agg_dens = []
        self.t_step = 0

class Field:

    def __init__(self):
        self.spr_fac = 0
        self.av_dist = []
        self.velocity = []

# class to instantiate aircraft
class Flight(Air_Object):
    pass

# class to instantiate waypoint
class Waypoint(Free_Air_Object):

    def coll_dens(self,flps):
        c=0
        for flp in flps:
            if flp in self.con_rad:
                c+=1
        self.dens=c/4
        self.t_step+=1
        self.agg_dens+=[self.dens]

# class to instantiate obstruction
class Obstruction(Free_Air_Object):
    pass

class TMA(Field):
    pass


'''Global Variables'''
con_rad = []
ccrad = 0
gnum_sdf = [0,0,0,0]
hov_fli = 0


'''Collect Data'''
# calculate and collect density at waypoints
def wdens(wps,flps):

    for wp in wps:
        wp.con_rad=ccf.obj_radius(wp.size,wp.pos)
        wp.coll_dens(flps)

# to collect pos & dist of flights in flight agg_pos & agg_distn respectively
def col_dept(flights):
    for f in flights:
        f.collect_pos()
        f.collect_distn()

# collect number of flights moving per t_step
def col_fl_per_t(flights):
    ftl=[]

    # create list of all agg_pos
    t_agpos=[flight.agg_pos for flight in flights]
    # try..except to handle n which is beyond index for list elements
    for n in range(len(t_agpos[0])):
        ft=0
        for m in t_agpos:
            try:
                if m[n] != m[n+1]:
                    ft+=1
            except:
                pass
        ftl+=[ft]

    return ftl

def av_distn(tma,flights):
    # get total t_step of experiment
    maxt= max([fl.t_step for fl in flights])
    av_dist= []

    for n in range(maxt):
        t_distn = 0
        c = 0
        for f in flights:
            if f.t_step <= maxt:
                t_distn+=f.agg_distn[n]
                c+=1
        av_dist_t=t_distn/c
        av_dist+=[av_dist_t]
    tma.av_dist=av_dist

# average velocity
def vel_per_t(tma,flights):
    no_fl = col_fl_per_t(flights)
    velocity = [nf / len(flights) for nf in no_fl]
    tma.velocity = velocity


'''Plots'''
# to plot all the flight paths
def path_plots(fpaths):

    for fpath in fpaths:
        fpath.flight_path()
        x,y=fpath.fp
        plt.plot(x,y)

# to plot waypoints
def wp_plots(wpp):
    for wp in wpp:
        nwp= np.array(wp)
        x,y=nwp.T
        plt.plot(x,y,'r+')

# to plot obstructions
def obs_plots(obp):
    for ob in obp:
        nob = np.array(ob)
        x, y = nob.T
        plt.plot(x,y,'bo')


'''Display'''
# to display to user density at waypoints for each t_step
def disp_wp_dens(waypoints):
    n=0
    for w in waypoints:
        wdd = []
        t=0
        for wd in w.agg_dens:
            t+=1
            swdis=['time step:',t,'density:',wd]
            wdd+=[swdis]
        n+=1
        print('Waypoint',n,w.pos,':',wdd,'\n')

# to display average distance of flights for each time step
def disp_av_distance(tma):
    savd = tma.av_dist

    avd = [['time step:', t, 'average distance=', a] for (a, t) in zip(savd, range(len(savd)))]

    print(avd)

# to display all results
def display_results(tma,flights,waypoints):
    for f in flights:
        print('total time steps:',f.t_step,', flight aggregate positions:',f.agg_pos)

    disp_wp_dens(waypoints)
    disp_av_distance(tma)


'''User Interaction'''
# to split flights from a direction to subdir (e.g north to ne & nw)
def split_num(num):
    fir = num // 2
    sec = num - fir

    sn = [fir, sec]
    return sn

def inst_flights(num_flights,dpx,dpy,dest):
    # arrange data placement in rand function
    if dpx < 1:
        dpx=[dpx,0]
    else:
        dpx=[0,dpx]

    if dpy < 0:
        dpy=[dpy,0]
    else:
        dpy=[0,dpy]

    flights=[Flight(dept=[random.randint(*dpx),random.randint(*dpy)],dest=dest)
             for _ in range(1,num_flights+1) if num_flights >= 1]
    return flights

# func for creating flights by subdir
def _nw_dep_flights(nw,nwd,dest):
    fl=inst_flights(nw, -nwd, nwd, dest)
    return fl

def _ne_dep_flights(ne,ned,dest):
    fl=inst_flights(ne, ned, ned, dest)
    return fl

def _sw_dep_flights(sw,swd,dest):
    fl=inst_flights(sw, -swd, -swd, dest)
    return fl

def _se_dep_flights(se,sed,dest):
    fl=inst_flights(se, sed, -sed, dest)
    return fl

# func to create flights
def create_flights(north,south,east,west,dest,spread):
    global gnum_sdf

    num_sdf=[north,south,east,west]
    gnum_sdf = [a + b for a, b in zip(num_sdf, gnum_sdf)]

    spn = split_num(north)
    ospn = split_num(gnum_sdf[0])
    n=_nw_dep_flights(spn[0],ospn[0]*spread,dest) + _ne_dep_flights(spn[1],ospn[1]*spread,dest)

    sps = split_num(south)
    osps = split_num(gnum_sdf[1])
    s=_se_dep_flights(sps[0],osps[0]*spread,dest) + _sw_dep_flights(sps[1],osps[1]*spread,dest)

    spe = split_num(east)
    ospe = split_num(gnum_sdf[2])
    e=_ne_dep_flights(spe[0],(ospn[1]+ospe[0])*spread,dest) + _se_dep_flights(spe[1],(osps[0]+ospe[1])*spread,dest)

    spw = split_num(west)
    ospw = split_num(gnum_sdf[3])
    w=_nw_dep_flights(spw[0],(ospn[0]+ospw[0])*spread,dest) + _sw_dep_flights(spw[1],(osps[1]+ospw[1])*spread,dest)

    flights = n + s + e + w
    return flights


'''Simulation'''
# simulate flights
def sim_iter(tma,flights,flights_pos,waypoints,obstructions):
    # collect departures in flight agg_pos
    col_dept(flights)

    wpp=[w.pos for w in waypoints]
    obp = [ob.pos for ob in obstructions]

    # write all waypoints in each flights waypoint variable
    for f in flights:
        f.way_p=[w.pos for w in waypoints]

    while True:
        print('flights_pos:', flights_pos)
        for flight in flights:
            ccf.conf_flight_movement(flights,flight,obp)
            flight.collect_pos()
            flight.collect_distn()

            if flight.pos in flight.way_p:
                flight.way_p.remove(flight.pos)

# collect flight densities at waypoint for each t_step
        wdens(waypoints,flights_pos)

# stop simulation when destination is reached
        flights_dest = [fd.dest for fd in flights]
        if flights_pos == flights_dest:
            break

    path_plots(flights)
    wp_plots(wpp)
    obs_plots(obp)
    plt.show()

    av_distn(tma,flights)
    vel_per_t(tma,flights)

# container function for flight simulation
def simulate(tma,flights,waypoints,obstructions):
    flights_pos=[f.pos for f in flights]

    sim_iter(tma,flights,flights_pos,waypoints,obstructions)

