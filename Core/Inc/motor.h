#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdio.h"

typedef struct
{
  float A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
  float A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
  float A2;          /**< The derived gain, A2 = Kd . */
  float state[3];    /**< The state array of length 3. */
  float Kp;          /**< The proportional gain. */
  float Ki;          /**< The integral gain. */
  float Kd;          /**< The derivative gain. */
} pid_instance;

void pid_init(pid_instance *S, uint8_t resetFlag)
{
  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float) 2.0 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if (resetFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3U * sizeof(float));
  }
}

float pid(pid_instance *S, float in)
{
  float out;

  /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
  out = (S->A0 * in) +
    (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

  /* Update state */
  S->state[1] = S->state[0];
  S->state[0] = in;

  if(out >= 3000) S->state[2] = 3000;
  else if(out <= -3000 ) S->state[2] = -3000;
  else S->state[2] = out;

  /* return to application */
  return (S->state[2]);
}

#endif /** __MOTOR_H **/