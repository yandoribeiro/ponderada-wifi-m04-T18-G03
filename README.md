# Ponderada - Mapa de Calor calculando dBm de sinal WIFI

Este projeto tem como propósito monitorar, de forma contínua, a Potência do Sinal de Rádio (RSSI) de uma rede Wi-Fi específica, utilizando um microcontrolador ESP32. As medições são publicadas em tempo real em uma plataforma de visualização por meio do protocolo MQTT (Message Queuing Telemetry Transport), no caso, utilizamos o broker Mosquitto por ser open source e já termos afinidade por está sendo utilizado no nosso projeto do Módulo.

A atividade se encerra com um experimento que simula o efeito de uma Gaiola de Faraday em um ambiente controlado, no caso, o elevador do Inteli. Durante a execução, registra-se a queda abrupta do sinal Wi-Fi (em dBm) e sua posterior recuperação, resultando em um gráfico contínuo que evidencia claramente o comportamento do sinal.

## Instalação e Execução (Local)

Para utilizar o Mosquitto localmente, é necessário instalá-lo e executá-lo como um serviço no computador, assegurando que esteja ativo e ouvindo na porta padrão 1883.

Instalação (Windows/macOS/Linux):
A instalação deve seguir as orientações fornecidas na documentação oficial do Mosquitto, de acordo com o sistema operacional utilizado.

Execução:
Após a instalação, verifique se o serviço do Mosquitto está em execução. Uma vez ativo, o broker ficará responsável por receber e encaminhar as conexões dos clientes — neste caso, o ESP32 e o dashboard — pela porta 1883.

**Teste de Conectividade:**

Para confirmar que o broker está funcionando corretamente, utilize as ferramentas de linha de comando fornecidas pelo Mosquitto, como mosquitto_sub e mosquitto_pub, que permitem testar a publicação e a assinatura de tópicos.

```
# Terminal 1: Assinante (Simula o Dashboard recebendo dados)
mosquitto_sub -h localhost -t /inteli/esp32/sinal_wifi

# Terminal 2: Publicador (Simula o ESP32 enviando um dado)
mosquitto_pub -h localhost -t /inteli/esp32/sinal_wifi -m "-55"
```

## Código utilizado

O código desenvolvido para o ESP32 segue uma abordagem de Programação Orientada a Objetos (POO), utilizando um ponteiro para gerenciar a instância do cliente MQTT. Essa estrutura permite encapsular de forma organizada toda a lógica relacionada à conexão com a rede Wi-Fi, ao gerenciamento do cliente MQTT e ao processo de publicação das medições.

``` jsx
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char *WIFI_SSID = "Inteli.Iot"; 
const char *WIFI_PASS = "%(Yk(sxGMtvFEs.3";
const char* MQTT_SERVER = "10.128.0.5";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_RSSI_Client_POO";

const char* MQTT_TOPIC_PUBLISH = "inteli/esp32/sinal_wifi";

const long PUBLISH_INTERVAL_MS = 1000;

WiFiClient espClient;

class RssiMqttClient {
private:
    PubSubClient* mqttClient;
    long lastPublishTime = 0;

    void connectWiFi() {
        Serial.print("Conectando a ");
        Serial.println(WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.println("\nWiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }

    void reconnectMQTT() {
        while (!mqttClient->connected()) {
            Serial.print("Conectando ao MQTT...");
            if (mqttClient->connect(MQTT_CLIENT_ID)) {
                Serial.println("conectado!");
            } else {
                Serial.print("falhou, rc=");
                Serial.println(mqttClient->state());
                delay(3000);
            }
        }
    }

public:
    RssiMqttClient(WiFiClient& client) {
        mqttClient = new PubSubClient(client);
        Serial.begin(115200);
        delay(10);
    }

    ~RssiMqttClient() {
        delete mqttClient;
    }

    void begin() {
        connectWiFi();
        mqttClient->setServer(MQTT_SERVER, MQTT_PORT);
    }

    void handleLoop() {
        if (!mqttClient->connected()) {
            reconnectMQTT();
        }

        mqttClient->loop();

        long now = millis();
        if (now - lastPublishTime > PUBLISH_INTERVAL_MS) {
            lastPublishTime = now;
            publishRssi();
        }
    }

    void publishRssi() {
        long rssi = WiFi.RSSI();

        StaticJsonDocument<100> doc;
        doc["rssi"] = rssi;

        String payload;
        serializeJson(doc, payload);

        Serial.print("Enviando JSON: ");
        Serial.println(payload);

        if (mqttClient->publish(MQTT_TOPIC_PUBLISH, payload.c_str())) {
            Serial.println("Publicado!");
        } else {
            Serial.println("Falha ao publicar!");
        }
    }
};

RssiMqttClient rssiClient(espClient);

void setup() {
    rssiClient.begin();
}

void loop() {
    rssiClient.handleLoop();
}
```

## Dashboard

O dashboard foi desenvolvido em Python utilizando duas bibliotecas principais: Paho-MQTT, responsável por subscrever e receber os dados publicados pelo ESP32, e Plotly Dash, utilizada para construir a visualização gráfica da série temporal, atualizada automaticamente a cada 2 segundos.

Quando o Mosquitto é executado localmente, o parâmetro MQTT_SERVER deve ser configurado como localhost, garantindo que o dashboard se conecte corretamente ao broker.

``` jsx
import json
import paho.mqtt.client as mqtt
from collections import deque
import dash
from dash import dcc, html
from dash.dependencies import Input, Output


BROKER_IP = "10.128.0.5"
BROKER_PORT = 1883
TOPIC = "inteli/esp32/sinal_wifi"

# Armazena últimas 200 leituras de RSSI
rssi_history = deque(maxlen=200)


def on_connect(client, userdata, flags, rc):
    print("Conectado ao broker:", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode("utf-8")
        data = json.loads(payload_str)

        if "rssi" in data:
            rssi_value = int(data["rssi"])
            rssi_history.append(rssi_value)
            print(f"RSSI recebido: {rssi_value}")

    except Exception as e:
        print("Erro ao processar JSON:", e)

# Inicializa o cliente MQTT
client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_IP, BROKER_PORT)
client.loop_start()


app = dash.Dash(_name_)

app.layout = html.Div([
    html.H1("Dashboard de RSSI do ESP32", style={"textAlign": "center"}),

    dcc.Graph(id="rssi-graph"),

    dcc.Interval(
        id="update-interval",
        interval=1000,  # Atualiza a cada 1 segundo
        n_intervals=0
    ),

    html.Div(id="last-value", style={"textAlign": "center", "fontSize": "24px"})
])

@app.callback(
    [Output("rssi-graph", "figure"),
     Output("last-value", "children")],
    [Input("update-interval", "n_intervals")]
)
def update_graph(n):
    if len(rssi_history) == 0:
        return {
            "data": [],
            "layout": {"title": "Sem dados ainda..."}
        }, "Aguardando dados MQTT..."

    figure = {
        "data": [{
            "x": list(range(len(rssi_history))),
            "y": list(rssi_history),
            "type": "line",
            "name": "RSSI"
        }],
        "layout": {
            "title": "RSSI do ESP32 em Tempo Real",
            "xaxis": {"title": "Leitura"},
            "yaxis": {"title": "RSSI (dBm)"},
        }
    }

    last_value = f"Último RSSI recebido: {rssi_history[-1]} dBm"

    return figure, last_value

if _name_ == "_main_":
    print("Iniciando dashboard em http://localhost:8050")
    app.run(host="0.0.0.0", port=8050, debug=True)
```

## Gráfico do sinal (RSSI)

A análise do gráfico no dashboard permite observar como a intensidade do sinal Wi-Fi varia de acordo com o ambiente. O RSSI, medido em dBm, representa a força do sinal: valores menos negativos indicam maior intensidade.

### Teste fora do elevador

Quando o celular, que atuou como ponto de acesso, foi posicionado próximo ao ESP32, o RSSI manteve-se entre –40 e –52 dBm, indicando um sinal forte e estável. Quando o celular foi afastado apenas alguns centímetros, o valor caiu rapidamente para aproximadamente –80 a –90 dBm, demonstrando a sensibilidade imediata do sistema mesmo para deslocamentos muito pequenos.|

### Teste dentro do elevador

Ao levar o celular para dentro do elevador e fechar a porta, o comportamento do sinal tornou-se ainda mais evidente. Assim que o ambiente ficou isolado, o RSSI despencou até cerca de –82 dBm e permaneceu congelado nesse valor, sem novas atualizações.

Esse travamento indica que a estrutura metálica do elevador bloqueou completamente o sinal Wi-Fi, impedindo que novas leituras fossem enviadas ao dashboard. Na prática, o ESP32 perdeu comunicação com o ponto de acesso, e o gráfico só voltou a ser atualizado quando o celular saiu do elevador, momento em que o sinal retornou e a transmissão foi restabelecida.

## Vídeo de demonstração

O vídeo terá como finalidade ilustrar, de forma prática, o comportamento do sistema durante o experimento. Ele mostrará a leitura contínua do RSSI no dashboard, seguida do momento em que o celular entra no elevador, ponto em que o gráfico cai para cerca de –82 dBm e permanece congelado devido à perda completa de comunicação. Ao retirar o celular do elevador, o vídeo exibirá também o retorno imediato das atualizações, evidenciando de maneira visual o efeito de isolamento eletromagnético e a precisão do sistema em detectar essa interrupção.

<p>
    Vídeo de demonstração <br>
    <a href="https://drive.google.com/file/d/12tj3jiiKhlV9PKuPZEOc9q6MpBLOvz_g/view?usp=sharing" target="_blank">
        Clique para abrir o vídeo no Google Drive
    </a>
</p>

## Conclusão

Os testes realizados, incluindo o cenário de Baseline, o teste de distância e a simulação da Gaiola de Faraday no elevador, demonstraram a eficácia do sistema composto por ESP32, MQTT e dashboard em Python na análise em tempo real da intensidade do sinal Wi-Fi.

A queda abrupta do RSSI ao fechar a estrutura metálica do elevador, aproximando-se de –95 dBm, confirmou o comportamento esperado de uma Gaiola de Faraday, evidenciando a atenuação significativa do campo eletromagnético. A recuperação imediata do sinal ao remover o dispositivo do ambiente blindado reforça a sensibilidade e precisão do monitoramento.

De forma geral, o experimento validou plenamente a capacidade do projeto de captar e visualizar variações de radiofrequência de maneira contínua, confiável e responsiva.
