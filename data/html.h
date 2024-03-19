/*
 * html.h
 *
 *  Created on: Apr 15, 2022
 *      Author: sfdd
 */

#ifndef HTML_H_
#define HTML_H_

#define HTTPS_STARTUP_WEBPAGE \
"<!DOCTYPE html>" \
"<html>" \
    "<head>" \
        "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\"/>" \
        "<meta charset=\"utf-8\" />" \
	"<meta http-equiv=\"refresh\" content=\"%d; URL=https://%lu:%d\">" \
	"<title>Simple Sensor Server</title>" \
    "</head>" \
    "<body>" \
        "<div id=\"container\">" \
            "<h1>A Simple Sensor Server</h1>" \
	    "<h2>CY8CKIT-062-WiFi-BT</h2>" \
            "<p class=\"article\">" \
		"<table class=\"table1\" border=2>" \
		"<tr><th>Sensor</th><th> Temperature </th><th> Humidity </th><th> Pressure </th></tr>" \
		"<tr><th>Value</th><td align=right> %.2f C</td><td align=right>%.2f %%</td><td align=right>%.2f hPa</td></tr>" \
		"</table>" \
            "</p>" \
		"<table class=\"table2\" border=2>" \
		"<tr><td><form method=\"post\">" \
			"<input type=\"submit\" name=\"toggle_led\" value=\"Toggle LED\"/>" \
		"</form></td>" \
		"<td bgcolor=%s><font color=white>%s</font></td></tr>" \
		"</table>" \
        "</div>" \
    "</body>" \
"</html>"

#endif /* HTML_H_ */
