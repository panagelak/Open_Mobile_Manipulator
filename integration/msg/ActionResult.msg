##Usage: Not used in any topic, but should be used in every action integration related action.

##Description: a generic action result that is expected and required to be part of each ActionResult so that the action handlers can respond to the caller the kind of error or interruption faced.
string action_id
bool success

#Use the constants of ActionResultStatusConstants in the status field
string status
uint64 millisPassed