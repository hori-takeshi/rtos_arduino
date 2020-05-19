#ifndef _R2CA_APP_H_
#define _R2CA_APP_H_

//#define MULTI_ECHO_SERVER

#ifndef MULTI_ECHO_SERVER
#define R2CA_NUM_TASK 0
#else /* MULTI_ECHO_SERVER */
#define R2CA_NUM_TASK 1
#endif /* MULTI_ECHO_SERVER */

#endif /* _R2CA_APP_H_ */
