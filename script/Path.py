#!/usr/bin/env python

import math

class Zone:
    def __init__(self, tx, ty, xi = True, yi = True):
        self.transX = tx
        self.transY = ty
        self.xIsLarger = xi
        self.yIsLarger = yi

    def checkX(self, x):
        if self.xIsLarger:
            if x>self.transX: return True
            else: return False
        else: 
            if x<self.transX: return True
            else: return False

    def checkY(self, y):
        if self.yIsLarger:
            if y>self.transY: return True
            else: return False
        else: 
            if y<self.transY: return True
            else: return False
    def ShouldTransit(self, x, y):
        if math.isnan(self.transX) and math.isnan(self.transY):
            return False
        elif math.isnan(self.transY):
            if self.checkX(x):
                return True
            else:
                return False
        elif math.isnan(self.transX):
            if self.checkY(y):
                return True
            else:
                return False
        else:
            if self.checkX(x) and self.checkY(y):
                return True
            else:
                return False

class ControlZone:
    def __init__(self, act_ty, cid, eid, tx, ty, xi = True, yi = True):
        self.border = Zone(tx,ty,xi,yi)
        self.res = (act_ty,cid,eid) #ty: 0->control in 1->merge in 2->control out

    def CheckZone(self, x, y):
        if self.border.ShouldTransit(x,y):
            return self.res
        else:
            return None

class Segment:

    def __init__(self, sid, tx, ty, xi = True, yi = True):
        self.track = Zone(tx,ty,xi,yi)
        self.segId = sid
        self.cZones = [] #WARNING: SEQUENCE MATERS

    def AppendControlBorderByRef(self, se):
        self.cZones.append(se)

    def AppendControlBorder(self, act_ty, cid, eid, tx, ty, xi = True, yi = True):
        self.cZones.append(ControlZone(act_ty,cid,eid,tx,ty,xi,yi))

    def ShouldTransit(self, x, y):
        return self.track.ShouldTransit(x,y)

    def CheckControl(self, x, y):
        for c in self.cZones:
            res = c.CheckZone(x,y)
            if res is not None:
                return res
        return None

class Path:

    def __init__(self):
        self.segments = []
        self.index = 0

    def AppendSegmentByRef(self, se):
        self.segments.append(se)

    def AppendSegment(self, sid, tx, ty, xi, yi):
        self.segments.append(Segment(sid,tx,ty,xi,yi))
    
    def CheckControl(self, x, y):
        return self.segments[self.index].CheckControl(x,y)

    def GetSegmentID(self, x, y):
        if len(self.segments) == 0:
            return -1
        else:
            if self.segments[self.index].ShouldTransit(x, y):
                self.index = self.index+1
                if self.index>=len(self.segments):
                    self.index = 0
                return self.segments[self.index].segId
            else:
                return self.segments[self.index].segId

    def GetSegmentIDNoCheck(self):
        if len(self.segments) == 0:
            return -1
        else:
            return self.segments[self.index].segId

def GetDefaultPath(index):
    res = Path()
    if index == 0:
        res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
        res.AppendSegment(2,-1.61,0.2042,True,True)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,-1.0715,float('nan'),True,True)

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,0.603929,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.295411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
        return res
    elif index == 1:
        res.AppendSegment(7,0.46,float('nan'),False,True)
        res.AppendSegment(8,float('nan'),-0.82,True,True)
        res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.698549,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True)

        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.603929,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(6,2.05,float('nan'),True,True)
        res.AppendSegment(12,float('nan'),-0.854,True,False)
        res.AppendSegment(13,2.00304,float('nan'),False,True)
        return res
    else:
        print "No default path id found"
        return None
     
