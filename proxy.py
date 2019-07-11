import threading
from flask import Flask
from flask_cors import CORS

import rospy
from std_srvs.srv import Empty

threading.Thread(target=lambda: rospy.init_node(
    'collector_service_proxy', disable_signals=True)).start()

app = Flask(__name__)
CORS(app)


@app.route('/', methods=['POST'])
def call_demand_service():
    rospy.wait_for_service("make_demand")
    make_demand = rospy.ServiceProxy("make_demand", Empty)
    rospy.loginfo("Calling the 'make_demand' service")
    make_demand()
    return ('', 204)


if __name__ == '__main__':
    app.run(port=8899)
