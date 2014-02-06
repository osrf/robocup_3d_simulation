# The perception message which each agent will receive on the
# observation of the field.

Header header
float32 field_of_view # radians diagonal 4x3 viewpoint. #TODO do we want to double parameterize this on horizontal and vertical? 
Landmark[] landmarks
AgentPose[] agents
