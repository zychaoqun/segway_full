import roslib;roslib.load_manifest('segway_dashboard')
import rospy

from segway_msgs.msg import *

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize
from .battery_widget import BatteryWidget

class SegwayDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        self._aux_bat = BatteryWidget("AuxPower")
        self._propulsion_bat = BatteryWidget("Propulsion")
        self._aux_sub = rospy.Subscriber('/segway/feedback/aux_power', AuxPower, self.aux_callback)
        self._propulsion_sub = rospy.Subscriber('/segway/feedback/propulsion', Propulsion, self.propulsion_callback)
        self._last_dashboard_message_time = rospy.get_time()
        self._system_charging = False

    def get_widgets(self):
        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context)], [self._aux_bat, self._propulsion_bat]]

    def aux_callback(self,msg):
        self._aux_bat.update_perc(msg.aux_soc[1])
        self._aux_bat.update_time(msg.aux_soc[1])
        if (0x1000 == (msg.aux_sys_status[1] & 0x1000)):
            self._aux_bat.set_charging(True)
            self._system_charging = True 
        else:
            self._aux_bat.set_charging(False)
            self._system_charging = False

    def propulsion_callback(self,msg):
        self._propulsion_bat.update_perc(msg.min_propulsion_battery_soc)
        self._propulsion_bat.update_time(msg.min_propulsion_battery_soc)
        self._propulsion_bat.set_charging(self._system_charging)
        
    def shutdown_dashboard(self):
        self._aux_sub.unregister()
        self._propulsion_sub.unregister()
