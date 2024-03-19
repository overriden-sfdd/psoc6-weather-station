#include "wifi_config.h"

/********************************************************************************
 * Function Name: wifi_connect
 ********************************************************************************
 * Summary:
 *  The device associates to the Access Point with given SSID, PASSWORD, and SECURITY
 *  type. It retries for MAX_WIFI_RETRY_COUNT times if the Wi-Fi connection fails.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the Wi-Fi connection is successfully
 *  established, a WCM error code otherwise.
 *
 *******************************************************************************/
cy_rslt_t wifi_connect(void)
{
	cy_rslt_t result = CY_RSLT_SUCCESS;
	uint32_t retry_count = 0;
	cy_wcm_connect_params_t connect_param = {0};
	cy_wcm_config_t wcm_config = {.interface = CY_WCM_INTERFACE_TYPE_STA};

	result = cy_wcm_init(&wcm_config);

	if (CY_RSLT_SUCCESS == result)
	{
		printf("Wi-Fi initialization is successful\n");
		memcpy(&connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
		memcpy(&connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
		connect_param.ap_credentials.security = WIFI_SECURITY_TYPE;
		printf("Join to AP: %s\n", connect_param.ap_credentials.SSID);

		/*
		 * Connect to Access Point. It validates the connection parameters
		 * and then establishes connection to AP.
		 */
		for (retry_count = 0; retry_count < MAX_WIFI_RETRY_COUNT; retry_count++)
		{
			result = cy_wcm_connect_ap(&connect_param, &ip_addr);

			if (CY_RSLT_SUCCESS == result)
			{
				printf("Successfully joined Wi-Fi network %s\n", connect_param.ap_credentials.SSID);

				if (CY_WCM_IP_VER_V4 == ip_addr.version)
				{
					printf("Assigned IP address: %s\n", ip4addr_ntoa((const ip4_addr_t *)&ip_addr.ip.v4));
				}
				else if (CY_WCM_IP_VER_V6 == ip_addr.version)
				{
					printf("Assigned IP address: %s\n", ip6addr_ntoa((const ip6_addr_t *)&ip_addr.ip.v6));
				}

				break;
			}

			printf("Failed to join Wi-Fi network. Retrying...\n");
		}
	}

	return result;
}
