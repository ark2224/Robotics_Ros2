#!/usr/bin/env python3
# coding: UTF-8
import sys
l111111_opy_ = sys.version_info [0] == 2
l1lll11_opy_ = 2048
l111lll_opy_ = 7
def l11l1l1_opy_ (l1ll1_opy_):
    global l11lll1_opy_
    stringNr = ord (l1ll1_opy_ [-1])
    l1ll111_opy_ = l1ll1_opy_ [:-1]
    ll_opy_ = stringNr % len (l1ll111_opy_)
    l11llll_opy_ = l1ll111_opy_ [:ll_opy_] + l1ll111_opy_ [ll_opy_:]
    if l111111_opy_:
        l1l11ll_opy_ = unicode () .join ([l1l11l_opy_ (ord (char) - l1lll11_opy_ - (l111l1l_opy_ + stringNr) % l111lll_opy_) for l111l1l_opy_, char in enumerate (l11llll_opy_)])
    else:
        l1l11ll_opy_ = str () .join ([chr (ord (char) - l1lll11_opy_ - (l111l1l_opy_ + stringNr) % l111lll_opy_) for l111l1l_opy_, char in enumerate (l11llll_opy_)])
    return eval (l1l11ll_opy_)
# l1lll1lll_opy_ l11111ll_opy_
# l1111lll_opy_ 4603 - l1111l11_opy_ 2023
import numpy
import rclpy
from rclpy.node import Node
from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData
class l111l1l1_opy_(Node):
    def __init__(self):
        super().__init__(l11l1l1_opy_ (u"ࠧࡦࡵࡷ࡭ࡲࡧࡴࡰࡴࠪ࢈"))
        self.l1lllllll_opy_ = self.create_publisher(RobotPose, l11l1l1_opy_ (u"ࠣ࠱ࡪࡸࡤࡶ࡯ࡴࡧࡢࡩࡸࡺࡩ࡮ࡣࡷࡩࡤࡍࡲࡢࡦ࡬ࡲ࡬ࠨࢉ"), 1)
        self.l1llll1ll_opy_ = self.create_publisher(RobotPose, l11l1l1_opy_ (u"ࠤ࠲࡫ࡹࡥ࡯ࡥࡱࡰࡩࡹࡸࡹࡠࡉࡵࡥࡩ࡯࡮ࡨࠤࢊ"), 1)
        self.x = numpy.zeros((3,1))
        self.l1lllll1l_opy_ = numpy.zeros(3,)
        self.l111l11l_opy_ = numpy.zeros((3,3))
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005
        self.l11lll11_opy_ = 0.05
        self.l1llllll1_opy_ = self.create_subscription(SensorData, l11l1l1_opy_ (u"ࠥ࠳ࡸ࡫࡮ࡴࡱࡵࡣࡩࡧࡴࡢࠤࢋ"), self.l1111ll1_opy_, 1)
    def l11111l1_opy_(self, l1111111_opy_):
        self.l1lllll1l_opy_[0] = self.l1lllll1l_opy_[0] + self.l11lll11_opy_ * l1111111_opy_.vel_trans * numpy.cos(self.l1lllll1l_opy_[2])
        self.l1lllll1l_opy_[1] = self.l1lllll1l_opy_[1] + self.l11lll11_opy_ * l1111111_opy_.vel_trans * numpy.sin(self.l1lllll1l_opy_[2])
        self.l1lllll1l_opy_[2] = self.l1lllll1l_opy_[2] + self.l11lll11_opy_ * l1111111_opy_.vel_ang
    def l1llll1l1_opy_(self, l1111111_opy_):
        l1llll111_opy_ = numpy.zeros((3,1))
        l1llll111_opy_[0] = self.x[0] + self.l11lll11_opy_ * l1111111_opy_.vel_trans * numpy.cos(self.x[2])
        l1llll111_opy_[1] = self.x[1] + self.l11lll11_opy_ * l1111111_opy_.vel_trans * numpy.sin(self.x[2])
        l1llll111_opy_[2] = self.x[2] + self.l11lll11_opy_ * l1111111_opy_.vel_ang
        l1lll11ll_opy_ = numpy.identity(3)
        l1lll11ll_opy_[0,2] = -numpy.sin(self.x[2]) * l1111111_opy_.vel_trans * self.l11lll11_opy_
        l1lll11ll_opy_[1,2] =  numpy.cos(self.x[2]) * l1111111_opy_.vel_trans * self.l11lll11_opy_
        l1lll1ll1_opy_ = numpy.dot(l1lll11ll_opy_, numpy.dot(self.l111l11l_opy_, numpy.transpose(l1lll11ll_opy_))) + self.V
        readings = []
        for i in range(0,len(l1111111_opy_.readings)):
            xr = l1llll111_opy_[0]
            l1lll1l11_opy_ = l1llll111_opy_[1]
            l1llll11l_opy_ = l1111111_opy_.readings[i].landmark.x
            l111ll11_opy_ = l1111111_opy_.readings[i].landmark.y
            if (numpy.sqrt( (xr-l1llll11l_opy_)*(xr-l1llll11l_opy_) + (l1lll1l11_opy_-l111ll11_opy_)*(l1lll1l11_opy_-l111ll11_opy_) ) > 0.1):
                readings.append(l1111111_opy_.readings[i])
        if len(readings) == 0:
            self.x = l1llll111_opy_
            self.l111l11l_opy_ = l1lll1ll1_opy_
            return
        H = numpy.zeros((2*len(readings),3))
        for i in range(0,len(readings)):
            xr = l1llll111_opy_[0]
            l1lll1l11_opy_ = l1llll111_opy_[1]
            l1llll11l_opy_ = readings[i].landmark.x
            l111ll11_opy_ = readings[i].landmark.y
            H[2*i+0, 0] = (xr-l1llll11l_opy_) / numpy.sqrt( (xr-l1llll11l_opy_)*(xr-l1llll11l_opy_) + (l1lll1l11_opy_-l111ll11_opy_)*(l1lll1l11_opy_-l111ll11_opy_) )
            H[2*i+0, 1] = (l1lll1l11_opy_-l111ll11_opy_) / numpy.sqrt( (xr-l1llll11l_opy_)*(xr-l1llll11l_opy_) + (l1lll1l11_opy_-l111ll11_opy_)*(l1lll1l11_opy_-l111ll11_opy_) )
            H[2*i+0, 2] = 0
            H[2*i+1, 0] =  (l111ll11_opy_-l1lll1l11_opy_) / ((l1llll11l_opy_-xr)*(l1llll11l_opy_-xr) + (l111ll11_opy_-l1lll1l11_opy_)*(l111ll11_opy_-l1lll1l11_opy_))
            H[2*i+1, 1] = -(l1llll11l_opy_-xr) / ((l1llll11l_opy_-xr)*(l1llll11l_opy_-xr) + (l111ll11_opy_-l1lll1l11_opy_)*(l111ll11_opy_-l1lll1l11_opy_))
            H[2*i+1, 2] = -1
        l111l1ll_opy_ = numpy.zeros((2*len(readings),1))
        for i in range(0,len(readings)):
            xr = l1llll111_opy_[0]
            l1lll1l11_opy_ = l1llll111_opy_[1]
            l111111l_opy_ = l1llll111_opy_[2]
            l1llll11l_opy_ = readings[i].landmark.x
            l111ll11_opy_ = readings[i].landmark.y
            l111l1ll_opy_[2*i+0] = readings[i].range - numpy.sqrt( (xr-l1llll11l_opy_)*(xr-l1llll11l_opy_) + (l1lll1l11_opy_-l111ll11_opy_)*(l1lll1l11_opy_-l111ll11_opy_) )
            l1lllll11_opy_ = readings[i].bearing - (numpy.arctan2(l111ll11_opy_-l1lll1l11_opy_, l1llll11l_opy_-xr) - l111111l_opy_)
            while l1lllll11_opy_ > numpy.pi:
                l1lllll11_opy_ -= 2*numpy.pi
            while l1lllll11_opy_ < -numpy.pi:
                l1lllll11_opy_ += 2*numpy.pi
            l111l1ll_opy_[2*i+1] = l1lllll11_opy_
        l111l111_opy_ = numpy.zeros((2*len(readings),2*len(readings)))
        for i in range(len(readings)):
            l111l111_opy_[2*i,2*i] = 0.1
            l111l111_opy_[2*i+1, 2*i+1] = 0.05
        S = numpy.dot( H, numpy.dot(l1lll1ll1_opy_, numpy.transpose(H)) ) + l111l111_opy_
        l1lll1l1l_opy_ = numpy.dot( numpy.dot(l1lll1ll1_opy_, numpy.transpose(H)), numpy.linalg.inv(S))
        self.x = l1llll111_opy_ + numpy.dot(l1lll1l1l_opy_, l111l1ll_opy_)
        self.l111l11l_opy_ = l1lll1ll1_opy_ - numpy.dot( l1lll1l1l_opy_, numpy.dot(H,l1lll1ll1_opy_) )
    def l1111ll1_opy_(self,l1111111_opy_):
        self.l11111l1_opy_(l1111111_opy_)
        l1l1111l_opy_ = RobotPose()
        l1l1111l_opy_.header.stamp = l1111111_opy_.header.stamp
        l1l1111l_opy_.pose.x = self.l1lllll1l_opy_[0]
        l1l1111l_opy_.pose.y = self.l1lllll1l_opy_[1]
        l1l1111l_opy_.pose.theta = self.l1lllll1l_opy_[2]
        self.l1llll1ll_opy_.publish(l1l1111l_opy_)
        self.l1llll1l1_opy_(l1111111_opy_)
        l1l1l11l_opy_ = RobotPose()
        l1l1l11l_opy_.header.stamp = l1111111_opy_.header.stamp
        l1l1l11l_opy_.pose.x = float(self.x[0])
        l1l1l11l_opy_.pose.y = float(self.x[1])
        l1l1l11l_opy_.pose.theta = float(self.x[2])
        self.l1lllllll_opy_.publish(l1l1l11l_opy_)
def main(args=None):
    rclpy.init(args=args)
    l1111l1l_opy_ = l111l1l1_opy_()
    rclpy.spin(l1111l1l_opy_)
if __name__ == l11l1l1_opy_ (u"ࠫࡤࡥ࡭ࡢ࡫ࡱࡣࡤ࠭ࢌ"):
   main()