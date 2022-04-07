/**
 * for ESP8266 differing between networks & clients: https://github.com/AlexLynd/ESP-Bug/
 * with packet filter : https://create.arduino.cc/projecthub/p99will/esp32-wifi-mac-scanner-sniffer-promiscuous-4c12f4
 * for ESP8266 : https://github.com/RicardoOliveira/ESPProLib
 * for ESP32 : https://github.com/ESP-EOS/ESP32-WiFi-Sniffer/blob/master/WIFI_SNIFFER_ESP32.ino
 * 
 * Espressif documentation : https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/index.html#wi-fi
 */

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "Arduino.h"

#define WIFI_CHANNEL_SWITCH_INTERVAL (500)
#define WIFI_CHANNEL_MAX (13)

uint8_t level = 0, channel = 1;

static wifi_country_t wifi_country = {.cc = "CN", .schan = 1, .nchan = 13}; //Most recent esp32 library struct

typedef struct
{
	unsigned frame_ctrl : 16;
	unsigned duration_id : 16;
	uint8_t addr1[6]; /* receiver address */
	uint8_t addr2[6]; /* sender address */
	uint8_t addr3[6]; /* filtering address */
	unsigned sequence_ctrl : 16;
	uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
	wifi_ieee80211_mac_hdr_t hdr;
	uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
	return ESP_OK;
}

void wifi_sniffer_init(void)
{
	const wifi_promiscuous_filter_t filt = {//Idk what this does
											.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
	ESP_ERROR_CHECK(esp_wifi_start());
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_filter(&filt);
	esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
	esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
	switch (type)
	{
	case WIFI_PKT_MGMT:
		return "MGMT";
	case WIFI_PKT_DATA:
		return "DATA";
	default:
	case WIFI_PKT_MISC:
		return "MISC";
	}
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{
	const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
	const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
	const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

	printf("PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
		   " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
		   " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
		   " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x\n",
		   wifi_sniffer_packet_type2str(type),
		   ppkt->rx_ctrl.channel,
		   ppkt->rx_ctrl.rssi,
		   /* ADDR1 */
		   hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
		   hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
		   /* ADDR2 */
		   hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
		   hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
		   /* ADDR3 */
		   hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
		   hdr->addr3[3], hdr->addr3[4], hdr->addr3[5]);
	if ((hdr->frame_ctrl & 0x00F0) == 0x0080)
	{
		printf("Beacon\n");
	}
}

// the setup function runs once when you press reset or power the board
void setup()
{
	Serial.begin(115200);
	delay(10);
	wifi_sniffer_init();
}

// the loop function runs over and over again forever
void loop()
{
	//Serial.print("inside loop");
	delay(1000); // wait for a second

	vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
	wifi_sniffer_set_channel(channel);
	channel = (channel % WIFI_CHANNEL_MAX) + 1;
}