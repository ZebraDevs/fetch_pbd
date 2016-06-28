from fetch_pbd_interaction.msg import Action
from mongo_msg_db_msgs.msg import Message
from mongo_msg_db_msgs.srv import Find, FindRequest
from mongo_msg_db_msgs.srv import Insert, InsertRequest
from mongo_msg_db_msgs.srv import Update, UpdateRequest
from rospy_message_converter import json_message_converter

import rospy


class ActionDatabase(object):
    def __init__(self, db_name, coll_name, find, insert, update):
        """Initialize this ActionDatabase.

        Args:
            db_name: string, the name of the database to use.
            coll_name: string, the name of the collection in the database to use.
            find: the rospy.ServiceProxy for searching the database.
            insert: the rospy.ServiceProxy for inserting into the database.
            update: the rospy.ServiceProxy for updating the database.
        """
        self._db_name = db_name
        self._collection_name = coll_name
        self._find = find
        self._insert = insert
        self._update = update
        self._MSG_TYPE = 'pr2_pbd_interaction/Action'

    @staticmethod
    def build_real():
        """Builds a real ActionDatabase for use on the robot.
        """
        db_name = 'pr2_pbd'
        coll_name = 'actions'
        find = rospy.ServiceProxy("mongo_msg_db/find", Find)
        insert = rospy.ServiceProxy("mongo_msg_db/insert", Insert)
        update = rospy.ServiceProxy("mongo_msg_db/update", Update)
        d = ActionDatabase(db_name, coll_name, find, insert, update)
        return d

    def insert_new(self, action_name):
        """Inserts a new action into the database.

        Args:
            action_name: string, the human-friendly name for this action.

        Returns:
            string, the ID of this action in the database.
        """
        req = InsertRequest()
        req.collection.db = self._db_name
        req.collection.collection = self._collection_name
        req.msg_type = self._MSG_TYPE
        action = Action()
        action.name = action_name
        req.json = json_message_converter.convert_ros_message_to_json(action)
        res = self._insert(req)
        return res.id

    def update(self, db_id, action):
        """Updates the action with the given ID.

        Args:
            db_id: The ID of this action in the database.
            action: The replacemen Action msg.
        """
        req = UpdateRequest()
        req.collection.db = self._db_name
        req.collection.collection = self._collection_name
        req.message = Message()
        req.message.id = db_id
        req.message.msg_type = self._MSG_TYPE
        req.message.json = json_message_converter.convert_ros_message_to_json(
            action)
        res = self._update(req)
        if res.matched_count == 0:
            rospy.logerr(
                'Action with ID {} not found, unable to update.'.format(db_id))

    def find(self, db_id):
        """Retrieves an action message with the given ID.

        Args:
            db_id: string, the ID in the database to look up.

        Returns: An Action msg, or None if the ID was not found.
        """
        req = FindRequest()
        req.collection.db = self._db_name
        req.collection.collection = self._collection_name
        req.id = db_id
        res = self._find(req)
        if res.matched_count == 0:
            rospy.logerr(
                'Action with ID {} not found, unable to retrieve.'.format(
                    db_id))
            return None
        else:
            msg_type = res.message.msg_type
            return json_message_converter.convert_json_to_ros_message(
                msg_type, res.message.json)
