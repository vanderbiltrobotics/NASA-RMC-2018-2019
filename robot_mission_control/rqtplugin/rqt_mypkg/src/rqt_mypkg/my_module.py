import os
import rospy
import rospkg
import threading
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtWidgets


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QtWidgets.QWidget()

        self.hard_stop.clicked[bool].connect(self._hardstop)
        self.soft_stop.clicked[bool].connect(self._softstop)
        self.control.clicked[bool].connect(self._switchmode)
        self.mode_val = 'Automatic'
        self.stopped = False
        self.mode.setText(self.mode_val)

        self.control_publisher = rospy.Publisher("control_mode", int, queue_size=0)
        self.rate = rospy.Rate(10)

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        threading.Thread(target=self.publish_manually, args=(0,)).start()

    def _hardstop(self):
        os.system("rosnode kill -a")
        self.mode.setText('Hard Stop')

    def _softstop(self):
        if not self.stopped:
            self.mode.setText('Soft Stop')
        else:
            pass

    def _switchmode(self):
        if self.mode_val == 'Automatic':
            self.mode_val = 'Manual'
        else:
            self.mode_val = 'Automatic'
        self.mode.setText(self.mode_val)

    # def shutdown_plugin(self):
    #     # unregister all publishers here
    #     pass
    #
    # def save_settings(self, plugin_settings, instance_settings):
    #     # save intrinsic configuration, usually using:
    #     # instance_settings.set_value(k, v)
    #     pass
    #
    # def restore_settings(self, plugin_settings, instance_settings):
    #     # restore intrinsic configuration, usually using:
    #     # v = instance_settings.value(k)
    #     pass

    def publish_manually(self):
        while not rospy.is_shutdown():
            if self.mode_val == 'Manual':
                self.control_publisher.publish(1)
            elif self.mode_val == 'Soft Stop':
                self.control_publisher.publish(2)
            else:
                self.control_publisher.publish(0)
            self.rate.sleep()
