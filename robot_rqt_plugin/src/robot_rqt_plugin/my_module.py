import os
import rospy
import rospkg
import threading
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtWidgets
from std_msgs.msg import Bool



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

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robot_rqt_plugin'), 'resource', 'MyPlugin.ui')
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

        # Sets up the functions to be called on clicking on buttons
        self._widget.hard_stop.clicked[bool].connect(self._hardstop)
        self._widget.soft_stop.clicked[bool].connect(self._softstop)
        self._widget.control.clicked[bool].connect(self._switchmode)

        # Sets up the variables and the text label
        self.mode_val = False
        self.soft_stop = False
        self.decide_text()

        # Sets the topics and the publishing rate
        self.control_publisher = rospy.Publisher("control_mode", Bool, queue_size=0)
        self.stop_publisher = rospy.Publisher("soft_stop", Bool, queue_size=0)
        self.rate = rospy.Rate(10)

        # Starts a thread to run in parallel publishing messages
        threading.Thread(target=self.publish_manually).start()

    def _hardstop(self):
        # Kills all the running nodes
        os.system("rosnode kill -a")
        # Updates the text label
        self._widget.hard.setText('All nodes killed')

    def _softstop(self):
        # Changes the value of the soft_stop variable
        # False means not stopped and True means stopped
        self.soft_stop = not self.soft_stop
        # Updates the text labels
        self.decide_text()

    def _switchmode(self):
        # Changes the value of the mode_val variable
        # False means automatic and True means manual
        self.mode_val = not self.mode_val
        # Updates the text labels
        self.decide_text()

    def publish_manually(self):
        # Runs till killed
        while not rospy.is_shutdown():
            # Publishes the value of the variables to the topics
            self.stop_publisher.publish(Bool(self.soft_stop))
            self.control_publisher.publish(Bool(self.mode_val))
            # Publishes at the set rate
            self.rate.sleep()

    def decide_text(self):
        # Sets the value for the text labels
        self._widget.mode.setText(['Manual' if self.mode_val else 'Automatic'][0])
        self._widget.soft.setText(['Enabled' if self.soft_stop else 'Disabled'][0])

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