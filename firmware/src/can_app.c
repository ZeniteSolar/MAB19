#include "can_app.h"

/**
 * @brief Prints a can message via usart
 */
inline void can_app_print_msg(can_t *msg)
{
#ifdef USART_ON
    usart_send_string("ID: ");
    usart_send_uint16(msg->id);
    usart_send_string(". D: ");

    for(uint8_t i = 0; i < msg->length; i++){
      usart_send_uint16(msg->data[i]);
      usart_send_char(' ');
    }

    usart_send_string(". ERR: ");
    can_error_register_t err = can_read_error_register();
    usart_send_uint16(err.rx);
    usart_send_char(' ');
    usart_send_uint16(err.tx);
    usart_send_char('\n');
#endif
}

/**
* @brief Manages the canbus application protocol
*/
inline void can_app_task(void)
{
    check_can();
 /*
    if(can_app_send_state_clk_div++ >= CAN_APP_SEND_STATE_CLK_DIV){
#ifdef USART_ON
        VERBOSE_MSG_CAN_APP(usart_send_string("state msg was sent.\n"));
#endif
        can_app_send_state();
        can_app_send_state_clk_div = 0;
    }

    if(can_app_send_pump_clk_div++ >= CAN_APP_SEND_PUMP)
    {
    #ifdef USART_ON
      VERBOSE_MSG_CAN_APP(usart_send_string("pump msg was sent.\n"));
    #endif
    can_app_send_pump();
    can_app_send_pump_clk_div = 0;
  }
*/
}

inline void can_app_send_state(void)
{
    can_t msg;
    msg.id                                  = CAN_FILTER_MSG_MAM17_STATE;
    msg.length                              = CAN_LENGTH_MSG_STATE;
    msg.flags.rtr = 0;

    msg.data[CAN_SIGNATURE_BYTE]            = CAN_SIGNATURE_SELF;
    msg.data[CAN_STATE_MSG_STATE_BYTE]      = (uint8_t) state_machine;
    msg.data[CAN_STATE_MSG_ERROR_BYTE]      = error_flags.all;

    can_send_message(&msg);
#ifdef VERBOSE_MSG_CAN_APP
    VERBOSE_MSG_CAN_APP(can_app_print_msg(&msg));
#endif
}

inline void can_app_send_pump(void)
{
    can_t msg;
    msg.id                                 = CAN_FILTER_MSG_MAM17_STATE;
    msg.length                              = CAN_LENGTH_MSG_STATE;
    msg.flags.rtr                          = 0;

    msg.data[CAN_SIGNATURE_BYTE]           = CAN_SIGNATURE_SELF;
    // Incluir status das bombas
    can_send_message(&msg);
}

void can_app_extractor_mic17_state(can_t *msg)
{
    if(msg->data[CAN_SIGNATURE_BYTE] == CAN_SIGNATURE_MIC17){
        if(msg->data[CAN_STATE_MSG_ERROR_BYTE]){
            //ERROR!!!
        }
    }
}

/**
 * @brief extract a pump message from mic17
 */
void can_app_extractor_mic17_pumps(can_t *msg)
{
    if(msg->data[CAN_SIGNATURE_BYTE] == CAN_SIGNATURE_MIC17)
    {
      system_flags.pump_on = bit_is_set(msg->data
        [CAN_MSG_MIC17_PUMPS_PUMPS_BYTE],
        CAN_MSG_MIC17_PUMPS_PUMP1_BIT);
    }
}

/**
 * @brief redirects a specific message extractor to a given message
 * @param *msg pointer to the message to be extracted
 */
inline void can_app_msg_extractors_switch(can_t *msg)
{
    if(msg->data[CAN_SIGNATURE_BYTE] == CAN_SIGNATURE_MIC17){
        can_app_checks_without_mic17_msg = 0;
        switch(msg->id)
        {
            case CAN_FILTER_MSG_MIC17_STATE:
            #ifdef USART_ON
                VERBOSE_MSG_CAN_APP(usart_send_string(" got a state msg from MIC17: "));
            #endif
                VERBOSE_MSG_CAN_APP(can_app_print_msg(msg));
                can_app_extractor_mic17_state(msg);
                break;

            case CAN_FILTER_MSG_MIC17_PUMPS:
            #ifdef USART_ON
                VERBOSE_MSG_CAN_APP(usart_send_string(" got a pump msg from MIC17: "));
            #endif
                VERBOSE_MSG_CAN_APP(can_app_print_msg(msg));
                can_app_extractor_mic17_pumps(msg);
                break;

            default:
            #ifdef USART_ON
                VERBOSE_MSG_CAN_APP(usart_send_string(" got a unknown msg from MIC17"));
            #endif
                VERBOSE_MSG_CAN_APP(can_app_print_msg(msg));
                break;
        }
    } // CAN_SIGNATURE_MIC17
}

/**
 * @brief Manages to receive and extract specific messages from canbus
 */
inline void check_can(void)
{
    // If no messages is received from mic17 for
    // CAN_APP_CHECKS_WITHOUT_MIC17_MSG cycles, than it go to a specific error state.
    //VERBOSE_MSG_CAN_APP(usart_send_string("checks: "));
    //VERBOSE_MSG_CAN_APP(usart_send_uint16(can_app_checks_without_mic17_msg));

#ifdef CAN_DEPENDENT
      if(can_app_checks_without_mic17_msg++ >= CAN_APP_CHECKS_WITHOUT_MIC17_MSG){
#ifdef USART_ON
        VERBOSE_MSG_CAN_APP(usart_send_string("Error: too many cycles without message.\n"));
#endif
        can_app_checks_without_mic17_msg = 0;
        error_flags.no_canbus = 1;
        set_state_error();
      }
#endif

    if(can_check_message()){
        can_t msg;
        if(can_get_message(&msg)){
            can_app_msg_extractors_switch(&msg);
        }
    }
}
