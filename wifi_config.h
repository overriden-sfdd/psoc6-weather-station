/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"
#include "cy_lwip.h"

/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_PASSWORD to match your Wi-Fi network
 * Credentials.
 */
#define WIFI_SSID                               "" // TODO: read SSID dynamically
#define WIFI_PASSWORD                           "" // TODO: read password dynamically

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                       CY_WCM_SECURITY_WPA2_AES_PSK

#define MAX_WIFI_RETRY_COUNT                     (3)

/* Holds the IP address obtained using Wi-Fi Connection Manager (WCM). */
extern cy_wcm_ip_address_t ip_addr;

cy_rslt_t wifi_connect(void);
