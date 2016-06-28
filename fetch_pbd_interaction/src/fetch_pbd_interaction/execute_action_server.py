from response import Response
from fetch_pbd_interaction.srv import ExecuteActionById, ExecuteActionByIdResponse
import rospy


class ExecuteActionServer(object):
    def __init__(self, interaction):
        """Initialize this server with the interaction class.

        Args:
            interaction: Interaction class to use.
        """
        self._interaction = interaction

    def serve(self, request):
        """Callback for serving ExecuteActionById requests.
        """
        self._interaction.switch_to_action_by_id(request.action_id)
        response_params = self._interaction._execute_action()
        response = Response(self._interaction._empty_response, response_params)
        response.respond()
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        timeout = rospy.Duration(60*5)
        while self._interaction.arms.is_executing():
            elapsed_time = rospy.Time.now() - start
            if elapsed_time > timeout:
                rospy.logwarn('PbD action did not finish after 5 minutes')
                break
            rate.sleep()

        response = ExecuteActionByIdResponse()
        return response
