from math import dist
import random

from matplotlib import pyplot as plt
import numpy as np

import confrules as ccf


'''Object Classes'''
# dept:departure, dest:destination, size:object_size_class
# pos:current_position, fir_pos:first_position, sec_pos:second_position
# agg_pos:aggregate_positions, t_step:time_step
class Air_Object:

    def __init__(self,dept,dest,size):
        self.dept = dept
        self.dest = dest
        self.tdes = []
        self.size = size
        self.con_rad = []
        self.ccrad = 0
        self.way_p = []
        self.pos = [dept[0],dept[1]]
        self.agg_pos = []
        self.sim_agg_pos = []
        self.t_step = 0
        self.avg_tnstime = 0
        self.sim_avg_tnstime = []
        self.distn = 0
        self.agg_distn = []
        self.sim_agg_distn = []

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

    def avg_transit_time(self):
        no_journ=self.agg_pos.count(self.dest)
        t=0
        ts=[]

        for f in self.agg_pos:
            t+=1
            if f==self.dest:
                ts+=[t]
                t=0

        if no_journ != 0:
            avg = sum(ts) / no_journ
        else:
            avg = 0
        self.avg_tnstime=avg

        return avg

# class for other objects and points in the air during flight
class Free_Air_Object:

    def __init__(self,size,pos):
        self.pos = pos
        self.size = size
        self.con_rad = []
        self.dens = 0
        self.agg_dens = []
        self.avg_dens = 0
        self.sim_avg_dens = []
        self.t_step = 0
        self.ob_mov = []
        self.agg_vel = []
        self.avg_vel = 0
        self.sim_avg_vel = []

    def coll_dens(self,flps):
        c=0
        cells=(2*self.size)**2
        for flp in flps:
            if flp in self.con_rad:
                c+=1
        self.dens=c/cells
        self.t_step+=1
        self.agg_dens+=[self.dens]
        self.agg_dens=[dens for dens in self.agg_dens ]

class Field:

    def __init__(self):
        self.spr_fac = 0
        self.av_dist = []
        self.sim_av_dist = []
        self.velocity = []

# class to instantiate aircraft
class Flight(Air_Object):
    pass

# class to instantiate waypoint
class Waypoint(Free_Air_Object):
    pass

# class to instantiate obstruction
class Obstruction(Free_Air_Object):
    pass

class Point(Free_Air_Object):
    pass

class TMA(Field):
    pass


'''Global Variables'''
con_rad = []
ccrad = 0
gnum_sdf = [0,0,0,0]
hov_fli = 0


'''Collect Data'''
# calculate and collect density at points
def dens(ps,flps,t_steps):

    for p in ps:
        p.coll_dens(flps)
        p.avg_dens=sum(p.agg_dens)/t_steps

# get con_rad of points
def p_con_rad(points):
    for p in points:
        p.con_rad = ccf.obj_radius(p.size, p.pos)

# to collect pos & dist of flights at departure in flight agg_pos & agg_distn respectively
def col_dept(flights):
    for f in flights:
        f.agg_pos=f.agg_pos+f.dept
        f.collect_distn()

def res_mov(ps,flights):

    for p in ps:
        for fl in flights:
            # container for res_mov(displacement) per t_step for flight at p
            mod_path = []
            fp=fl.agg_pos
            pc=p.con_rad
            for n in range(len(fp)):
                try:
                    # entry to p_rad
                    if fp[n+1] in pc and fp[n] not in pc:
                        c=0
                        en=fp[n+1]
                    if fp[n] not in pc:
                        mod_path+=[0]
                    if fp[n] in pc:
                        c+=1
                    # at exit from p_rad collect each disp. per t_step within p_rad
                    if fp[n+1] not in pc and fp[n] in pc:
                        ex=fp[n]
                        e= [nt-xt for (nt,xt) in zip(en,ex)]
                        e_ab= max(map(abs,e))
                        e_mod=[e_ab/c for _ in range(c)]
                        mod_path+=e_mod
                except:
                    pass
            # collect each flight disp. within p_rad
            p.ob_mov+=[mod_path]
        p.agg_vel=list(map(sum,p.ob_mov))

def av_distn(tma,flights):
    # get total t_step of experiment
    maxt= max([fl.t_step for fl in flights])
    av_dist= []

    # if statement is for flights that reach dest early and with repetitive dest
    for n in range(maxt):
        t_distn = 0
        c = 0
        for f in flights:
            t_distn+=f.agg_distn[n]
            c+=1
        av_dist_t=t_distn/c
        av_dist+=[av_dist_t]
    tma.av_dist=av_dist

# average velocity
def vel_per_t(ps,flights,t_steps):
    res_mov(ps,flights)
    for p in ps:
        p.avg_vel=sum(p.agg_vel)/t_steps
    # velocity = [nf / len(flights) for nf in no_fl]


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

def inst_flights(num_flights,dpx,dpy,dest,size):
    # arrange data placement in rand function
    if dpx < 1:
        dpx=[dpx,0]
    else:
        dpx=[0,dpx]

    if dpy < 0:
        dpy=[dpy,0]
    else:
        dpy=[0,dpy]

    flights=[Flight(dept=[random.randint(*dpx),random.randint(*dpy)],dest=dest,size=size)
             for _ in range(1,num_flights+1) if num_flights >= 1]
    return flights

# func for creating flights by subdir
def _nw_dep_flights(nw,nwd,dest,size):
    fl=inst_flights(nw, -nwd, nwd, dest, size)
    return fl

def _ne_dep_flights(ne,ned,dest,size):
    fl=inst_flights(ne, ned, ned, dest, size)
    return fl

def _sw_dep_flights(sw,swd,dest,size):
    fl=inst_flights(sw, -swd, -swd, dest, size)
    return fl

def _se_dep_flights(se,sed,dest,size):
    fl=inst_flights(se, sed, -sed, dest, size)
    return fl

# func to create flights
def create_flights(north,south,east,west,dest,size,spread):
    global gnum_sdf

    num_sdf=[north,south,east,west]
    gnum_sdf = [a + b for a, b in zip(num_sdf, gnum_sdf)]

    spn = split_num(north)
    ospn = split_num(gnum_sdf[0])
    n=_nw_dep_flights(spn[0],ospn[0]*spread,dest,size) + _ne_dep_flights(spn[1],ospn[1]*spread,dest,size)

    sps = split_num(south)
    osps = split_num(gnum_sdf[1])
    s=_se_dep_flights(sps[0],osps[0]*spread,dest,size) + _sw_dep_flights(sps[1],osps[1]*spread,dest,size)

    spe = split_num(east)
    ospe = split_num(gnum_sdf[2])
    e=_ne_dep_flights(spe[0],(ospn[1]+ospe[0])*spread,dest,size) + _se_dep_flights(spe[1],(osps[0]+ospe[1])*spread,dest,size)

    spw = split_num(west)
    ospw = split_num(gnum_sdf[3])
    w=_nw_dep_flights(spw[0],(ospn[0]+ospw[0])*spread,dest,size) + _sw_dep_flights(spw[1],(osps[1]+ospw[1])*spread,dest,size)

    flights = n + s + e + w
    return flights


'''Simulation'''
# simulate flights
def sim_iter(tma,flights,flights_pos,waypoints,obstructions,points,total_tstep):
    # collect departures in flight agg_pos and get points conflict dens
    col_dept(flights)
    p_con_rad(waypoints)
    p_con_rad(points)

    wpp=[w.pos for w in waypoints]
    obp = [ob.pos for ob in obstructions]

    # write all waypoints in each flights waypoint variable
    for f in flights:
        f.way_p=[w.pos for w in waypoints]

    flights_dest = [fd.dest for fd in flights]

    if total_tstep>1:
        for t in range(total_tstep):
            print('flights_pos:', flights_pos)
            for flight in flights:
                ccf.conf_flight_movement(flights, flight, obp)
                flight.collect_pos()
                flight.collect_distn()

                if flight.pos in flight.way_p:
                    flight.way_p.remove(flight.pos)

                # cycle flight
                if flight.pos==flight.dest:
                    flight.pos[0],flight.pos[1]=flight.dept[0],flight.dept[1]

        # collect flight densities at points for each t_step
            dens(waypoints, flights_pos, total_tstep)
            dens(points, flights_pos, total_tstep)

    elif total_tstep==1:
        while True:
            print('flights_pos:', flights_pos)
            for flight in flights:
                ccf.conf_flight_movement(flights,flight,obp)
                flight.collect_pos()
                flight.collect_distn()

                if flight.pos in flight.way_p:
                    flight.way_p.remove(flight.pos)

        # collect flight densities at points for each t_step
            dens(waypoints, flights_pos, total_tstep)
            dens(points, flights_pos, total_tstep)

        # stop simulation when destination is reached
            if flights_pos == flights_dest:
                break

    # get average transit time of flights
    for f in flights:
        f.avg_transit_time()

    path_plots(flights)
    wp_plots(wpp)
    obs_plots(obp)
    plt.show()

    av_distn(tma,flights)
    vel_per_t(points,flights,total_tstep)


# container function for simulation
def simulate(tma,flights,waypoints,obstructions,points,total_tsteps):
    flights_pos=[f.pos for f in flights]

    sim_iter(tma,flights,flights_pos,waypoints,obstructions,points,total_tsteps)
