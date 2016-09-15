# Dirction values can be -1 for counter-clockwise, 0 for no, amd 1 clockwise rotation
int8 direction

# Sum of direction values since the launch of the node
int64 integral

# TRUE when button is pressed, FALSE otherwise
bool is_pressed

# TRUE if the event was triggered by a push or a release of the button; FALSE otherwise
bool push_state_changed