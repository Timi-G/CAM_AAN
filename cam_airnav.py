from rules import Flight, Waypoint, Obstruction, TMA, Point,\
                            simulate, display_results, create_flights


'''create flight, waypoint & obstruction objects f1,f2,f3,w1,w2,ob1,ob2...
and include all objects needed in respective experiment list

for random generated flights, follow convention in example fr1,fr2...
args north, south, east, west defines number of flights with entry points/departure
in that direction. size is size of flight, spread is multiple by which flight is
scattered across tma and apace is to define a static perimeter.
spread must be 1 for aspace to have any effect.

set total_tstep=1 to simulate until destination is reached

for density around points,use p1.agg_dens,p2.agg_dens,w1.agg_dens....
get average density around points with p1.avg_dens,p2.avg_dens,w1.avg_dens....
get average velocity with p1.avg_vel,p2.avg_vel...
get average distance with tma.av_dist
get average transit time with f1.avg_transit_time, f2.avg_transit_time...
'''
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    tma = TMA()

    dest = [5,5]
    f1 = Flight(dept=[-7,3], dest=dest, size=2)
    f2 = Flight(dept=[14,3], dest=dest, size=1)
    f3 = Flight(dept=[12,0], dest=[3,8], size=3)
    f4 = Flight(dept=[13,9], dest=dest, size=1)
    f5 = Flight(dept=[5,5], dest=[10,3], size=2)
    f6 = Flight(dept=[1,1], dest=[15,2], size=1)

    fr1 = create_flights(north=1, south=3, east=2, west=0, dest=[5, 1], size=1, spread=1, aspace=50)
    fr2 = create_flights(north=1, south=0, east=0, west=4, dest=[7, 15], size=2, spread=1, aspace=50)
    fr3 = create_flights(north=2, south=3, east=0, west=5, dest=[17, 20], size=1, spread=1, aspace=50)

    w1 = Waypoint(pos=[-13,4], size=1)
    w2 = Waypoint(pos=[10,20], size=1)
    w3 = Waypoint(pos=[20,0], size=1)

    ob1 = Obstruction(pos=[14,7], size=1)
    ob2 = Obstruction(pos=[9,3], size=1)
    ob3 = Obstruction(pos=[-15,34], size=1)
    ob4 = Obstruction(pos=[4,18], size=1)

    p1 = Point(pos=[6,2], size=2)
    p2 = Point(pos=[10,5], size=3)

    flights = fr1+fr3+[f1,f2,f3,f4,f5]
    waypoints = [w2]
    obstructions = [ob1,ob2,ob3,ob4]
    points = [p1,p2]


# to run simulation
    simulate(tma,flights,waypoints,obstructions,points,total_tsteps=100)

    display_results(tma,flights,waypoints)
