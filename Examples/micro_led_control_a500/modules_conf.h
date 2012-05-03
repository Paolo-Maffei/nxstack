/* modules_conf.h generated */

#ifdef COMPILING_MODULE_C
extern portCHAR cipv6_init(buffer_t *buf);
extern portCHAR cipv6_handle( buffer_t *buf );
extern portCHAR cipv6_check( buffer_t *buf );

extern portCHAR cudp_init(buffer_t *buf);
extern portCHAR cudp_handle( buffer_t *buf );
extern portCHAR cudp_check( buffer_t *buf );

extern portCHAR dri_init(buffer_t *buf);
extern portCHAR dri_handle( buffer_t *buf );
extern portCHAR dri_check( buffer_t *buf );

extern portCHAR icmp_init(buffer_t *buf);
extern portCHAR icmp_handle( buffer_t *buf );
extern portCHAR icmp_check( buffer_t *buf );
uint8_t gateway_features;
extern portCHAR mac_init(buffer_t *buf);
extern portCHAR mac_handle( buffer_t *buf );
extern portCHAR mac_check( buffer_t *buf );

extern portCHAR nrp_init(buffer_t *buf);
extern portCHAR nrp_handle( buffer_t *buf );
extern portCHAR nrp_check( buffer_t *buf );

extern portCHAR nudp_init(buffer_t *buf);
extern portCHAR nudp_handle( buffer_t *buf );
extern portCHAR nudp_check( buffer_t *buf );

extern portCHAR nwk_manager_init( buffer_t *buf);
extern portCHAR nwk_manager_handle( buffer_t *buffer );
extern portCHAR nwk_manager_check( buffer_t *buf );

extern portCHAR rf_802_15_4_init(buffer_t *buf);
extern portCHAR rf_802_15_4_handle( buffer_t *buf );
extern portCHAR rf_802_15_4_check( buffer_t *buf );

extern portCHAR ssi_init(buffer_t *buf);
extern portCHAR ssi_handle( buffer_t *buf );
extern portCHAR ssi_check( buffer_t *buf );


module_t modules[] = {
#ifdef HAVE_CIPV6
  {cipv6_init, cipv6_handle, cipv6_check, 0, MODULE_CIPV6, 41, ADDR_802_15_4_LONG, 0 },
#endif
#ifdef HAVE_CUDP
  {cudp_init, cudp_handle, cudp_check, 0, MODULE_CUDP, 8, ADDR_NONE, 0 },
#endif
#ifdef HAVE_DRI
  {dri_init, dri_handle, dri_check, 0, MODULE_DRI, 0, ADDR_NONE, 0 },
#endif
#ifdef HAVE_ICMP
  {icmp_init, icmp_handle, icmp_check, 0, MODULE_ICMP, 16, ADDR_NONE, 0 },
#endif
#ifdef HAVE_MAC_15_4
  {mac_init, mac_handle, mac_check, 0, MODULE_MAC_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif
#ifdef HAVE_NRP
  {nrp_init, nrp_handle, nrp_check, 0, MODULE_NRP, 0, ADDR_NONE, 0 },
#endif
#ifdef HAVE_NUDP
  {nudp_init, nudp_handle, nudp_check, 0, MODULE_NUDP, 5, ADDR_802_15_4_PAN_LONG, 0 },
#endif
#ifdef HAVE_NWK_MANAGER
  {nwk_manager_init, nwk_manager_handle, nwk_manager_check, 0, MODULE_NWK_MANAGER, 0, ADDR_NONE, 0 },
#endif
#ifdef HAVE_RF_802_15_4
  {rf_802_15_4_init, rf_802_15_4_handle, rf_802_15_4_check, 0, MODULE_RF_802_15_4, 23, ADDR_802_15_4_PAN_LONG, 0 },
#endif
#ifdef HAVE_SSI
  {ssi_init, ssi_handle, ssi_check, 0, MODULE_SSI, 0, ADDR_NONE, 0 },
#endif
{0, 0, 0, 0, MODULE_NONE, 0, ADDR_NONE, 0 } };
#else
extern module_t modules[];
#endif
