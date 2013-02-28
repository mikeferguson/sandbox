#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "gpio.hpp"
typedef Gpio<GPIOD_BASE,1> act;

static err_t tcp_accept_callback(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static void tcp_conn_err_callback(void *arg, err_t err);

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *addr, u16_t port);

struct udp_pcb *my_udp = NULL;

// Initialize stuff needed for network application :
//   *  a udp connection listening on port 5048
int netapp_init()
{ 
  // UDP connection
  my_udp = udp_new();
  if (my_udp == NULL)
  {
  return 0;
  }

  if (udp_bind(my_udp, IP_ADDR_ANY, 5048) != ERR_OK)
  {
  return 0;
  }

  udp_recv(my_udp, udp_recv_callback, NULL);

  return 1;
}

static void udp_recv_callback(void *arg, struct udp_pcb *udp, struct pbuf *p, struct ip_addr *ipaddr, u16_t port)
{
  act::high();
  uint8_t* data = (uint8_t*) p->payload;
  struct pbuf *p_send  = pbuf_alloc(PBUF_TRANSPORT, 3, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    /**x++ = 0xff; */
    *x++ = 'B'; *x++ = 'O'; *x++ = 'T';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '1';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '2';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '3';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '4';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '5';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '6';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '7';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '8';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  /* send again */
  p_send  = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p_send != NULL)
  {
    unsigned char * x = (unsigned char*) p_send->payload;
    *x++ = 'M'; *x++ = 'C'; *x++ = 'B'; *x++ = '9';
    udp_sendto(udp, p_send, ipaddr, port);
    pbuf_free(p_send);
  }

  // TODO: use packets for good, not evil
  pbuf_free(p);
  act::low();
}


