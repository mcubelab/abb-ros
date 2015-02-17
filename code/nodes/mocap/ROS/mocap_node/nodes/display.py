
#!/usr/bin/env python
PKG = 'mocap_node'
import roslib; roslib.load_manifest(PKG)

import rospy, numpy, time

from mocap_node.msg import MocapPt
from std_msgs.msg import String


from subprocess import *

rotmat = numpy.matrix(numpy.loadtxt('trans.matrix'))
ers = numpy.loadtxt('error_table', delimiter=',')

def error(pt):
    n = 1; zr = 1e-3
    b = ((ers[:,0:3]-pt)**2)
    d = ((b[:,0]+b[:,1]+b[:,2])**.5)
    d[d==0] = zr; rd = 1/d; rd = rd**n; 
    ans = (sum(ers[:,3]*rd)/sum(rd))
    return ans

def line2pt(line):
    b = line.strip()
    v = numpy.matrix([[float(b.split(',')[1].split("[")[1])], [float(b.split(',')[2])], [float(b.split(',')[3].split(']')[0])], [0]])
    v *= 1000; v[3][0] = 1; v = rotmat*v
    pt = [float(v[0][0]), float(v[1][0]), float(v[2][0])]
    return pt

def mocap_dump():
    pub = rospy.Publisher('mocap_marker_data', MocapPt)
    rospy.init_node('mocap')
    msg = MocapPt()
    #msg.XYZ = [1,2,3]
    #msg.error = [1.0]

    try:
        p = Popen(["./sedscript.sh"], stdout=PIPE)
        print 'opened parser'
        while not rospy.is_shutdown():
            try: pc = int(p.stdout.readline().strip())
            except: print "nope"
            if (pc == 0): pass
            else:
                
                ptsl = []; errl = []
                for i in range(0,pc):                
                    pt = line2pt(p.stdout.readline())
                    pterror = error(pt)
                    ptsl.append(pt)
                    errl.append(pterror)
                msg.pcount = pc                
                msg.XYZ = numpy.array(ptsl).flatten()
                msg.error = errl
                #print 'ptsl: ', numpy.array(ptsl).flatten()
                #print 'error: ', errl
                #print 'pcount: ', pc
                pub.publish(msg)
    except:
        print 'exit main loop'
        p.terminate()


if __name__ == '__main__':
    try:
        mocap_dump()
    except rospy.ROSInterruptException: pass


