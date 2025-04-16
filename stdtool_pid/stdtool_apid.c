/**
  ******************************************************************************
  * @file    apid.c
  * @author  Arthur
  * @version v2
  * @date    2018-5-18
  * @brief   
             2018-9-11
             add sample time, not tested.
  *          2018-11-17
             solve the problem of time period, and rewrite all the comments.

  ******************************************************************************
  */

/*******************************************************************************
************************************ Includes **********************************
*******************************************************************************/
#include "./stdtool_apid.h"
#include <math.h>

#define F32 float


/*******************************************************************************
******************************** Private define ********************************
*******************************************************************************/
/* the pid sample period in Second 
2018-11-17 do not use it , for if you set a period, the whold apid module will
be only sutiable for only one task, that is not my design.
static const float pid_samping_time = AGV_CORE_PERIOD_MS * 0.001f;
*/

/*******************************************************************************
  * @brief  saturation of a variable between min and max, for float.
  *
  * @param  x = input
            min = min saturation setting
            max = max saturation setting
  *
  * @retval f32 the saturated value
  * @notte  none
  *****************************************************************************/
float stdtool_fsat(float x, float min, float max)
{
    if(x < min){
        x = min;
        return x;
    }
    else if (x > max){
        x = max;
        return x;
    }
    else{
        return x;
    }
}

/*******************************************************************************
  * @brief  advancde pid controller (with four steps of limitation) 
  *
  * @param  apid = PID struct ptr.
  *
  * @retval none
  * @notte  none
  *****************************************************************************/
void Advanced_PID_Controller(APID_STRU * apid)
{
    //the output evertually
    F32 aout;                                               
    
    //compute the err between sp and pv
    apid->err = apid->sp - apid->pv;                        

    /*
    THE PROPORTIONAL PRAT
    */
    apid->part_p = apid->err * apid->KP;                   
    /* amp limit for p part */
    apid->part_p =  stdtool_fsat(apid->part_p, apid->P_LIM_DN, apid->P_LIM_UP);


    /* 
    THE INTEGRATION PART
    */
    /* if err is small enough, do the i caculation */
    if(stdtool_fabs(apid->err) < apid->I_DEADZONE){                  
        /* add up to the i part, the history value */
        apid->part_i += (apid->err * apid->KI);             
    }
    /* now seeing the err is very big */
    else {                                                 
        /* let the i part go decaying */
        apid->part_i *= apid->I_FALLRATE;                   
    }
    /* amp limit for i part */
    apid->part_i = stdtool_fsat(apid->part_i, apid->I_LIM_DN, apid->I_LIM_UP); 

        
    /*
    THE DIFFERENTIATION PART
    */
    /* calculate the d value, using diff of this time minus diff of last time. */
    apid->part_d = (apid->err - apid->err_last) * apid->KD; 
    /* amp limit of d part*/
    apid->part_d = stdtool_fsat(apid->part_d, apid->D_LIM_DN , apid->D_LIM_UP);
    
    /* sum of three part, and prepare for outputing*/
    aout = apid->part_p + apid->part_i + apid->part_d;      
    /* amp limit for the outputing */
    aout = stdtool_fsat(aout, apid->TOTAL_LIM_DN, apid->TOTAL_LIM_UP);
    /* the output is here */
    apid->out = aout;
    /* update err last data */
    apid->err_last = apid->err;                             
}



/*******************************************************************************
  * @brief  reset pid data struct
  *
  * @param  apid = PID struct pointer
  *
  * @retval none
  * @notte  none
  *****************************************************************************/
void Advanced_PID_Para_Reset(APID_STRU * apid)
{
    apid->err_last=0.0f;
    
    apid->part_i=0.0f;
}

/********************************* end of file ********************************/

