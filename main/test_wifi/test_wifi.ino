// TEST: Минимальный тест WiFi
// Ничего кроме WiFi: ни датчиков, ни таймеров, ни серв.
// Цель: проверить, переживает ли плата включение WiFi-радио.
//
// Если ДАЖЕ ЭТОТ скетч перезагружается (ets Jul 29... после "WiFi ON") -
// проблема в питании/железе: кабель USB, стабилизатор платы,
// подключённая периферия. Отключай всё лишнее по одному.
//
// Если этот скетч работает (статусы печатаются каждые 0.5 сек) -
// проблема во взаимодействии с остальным кодом, копаем скетч.

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== WIFI MINIMAL TEST ===");
  Serial.println("Turning WiFi ON in 2 sec...");
  delay(2000);

  Serial.println("WiFi ON");
  WiFi.mode(WIFI_STA);
  WiFi.begin("test_network", "12345678");  // сеть может не существовать - не важно
  Serial.println("WiFi.begin() OK - radio survived!");
}

void loop() {
  // 3 = connected, 1 = no ssid available, 6 = disconnected
  Serial.print("status=");
  Serial.print(WiFi.status());
  Serial.print(" heap=");
  Serial.println(ESP.getFreeHeap());
  delay(500);
}
