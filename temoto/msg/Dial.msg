# dirction values are -1, 0, 1 depending on the actual rotation
int8 direction

# sum of direction values since the start of the node
int64 integral

# 0 means button is depressed; 1 that it is pressed
int8 pressed

# true if button was pushed or released; false otherwise
bool push_ev_occured