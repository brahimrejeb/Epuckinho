#ifndef SEARCH_H_
#define SEARCH_H_
#define STOP_SPEED 0 // Halt speed

void start_search(void);
void unblock_ball(uint8_t* counter);
bool get_no_goal(void);
void set_no_goal(bool val);

#endif /* SEARCH_H_ */
