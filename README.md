# Ponderada - Mapa de Calor calculando dBm de sinal WIFI

Este projeto tem como propósito monitorar, de forma contínua, a Potência do Sinal de Rádio (RSSI) de uma rede Wi-Fi específica, utilizando um microcontrolador ESP32. As medições são publicadas em tempo real em uma plataforma de visualização por meio do protocolo MQTT (Message Queuing Telemetry Transport), no caso, utilizamos o broker Mosquitto por ser open source e já termos afinidade por está sendo utilizado no nosso projeto do Módulo.

A atividade se encerra com um experimento que simula o efeito de uma Gaiola de Faraday em um ambiente controlado, no caso, o elevador do Inteli. Durante a execução, registra-se a queda abrupta do sinal Wi-Fi (em dBm) e sua posterior recuperação, resultando em um gráfico contínuo que evidencia claramente o comportamento do sinal.

**Uso e Configuração do Mosquitto Broker**

O Mosquitto atua como o Broker MQTT, o intermediário que recebe as mensagens do ESP32 (Publisher) e as envia para o Dashboard Python (Subscriber).

📝 Documentação do Projeto IoT: Monitoramento de Sinal Wi-Fi (RSSI) com ESP32 e MQTT
1. 💡 Contexto da Atividade
Este projeto tem como objetivo principal monitorar a Potência do Sinal de Rádio (RSSI) de uma rede Wi-Fi específica, utilizando um microcontrolador ESP32, e publicar esses dados em tempo real em uma plataforma de visualização através do protocolo MQTT (Message Queuing Telemetry Transport).

A atividade culmina na realização de um experimento para simular o efeito da Gaiola de Faraday em um ambiente controlado (o elevador do Inteli), registrando a queda abrupta e posterior recuperação do sinal Wi-Fi (dBm) em um gráfico contínuo.

Shutterstock
Explorar

2. ⚙️ Uso e Configuração do Mosquitto Broker (Local)
O Mosquitto atua como o Broker MQTT, o intermediário que recebe as mensagens do ESP32 (Publisher) e as envia para o Dashboard Python (Subscriber).

Instalação e Execução (Local):

O Mosquitto deve ser instalado e iniciado como um serviço no seu computador, garantindo que ele esteja escutando na porta padrão 1883.

Instalação (Windows/macOS/Linux): Siga as instruções específicas para o seu sistema operacional na documentação oficial do Mosquitto.

Execução: Após a instalação, garanta que o serviço Mosquitto esteja rodando. O broker escutará as conexões de clientes (ESP32 e Dashboard) na porta 1883.

Teste de Conectividade:

Use as ferramentas de linha de comando do Mosquitto (mosquitto_sub e mosquitto_pub) para confirmar que o broker está funcionando.

```
# Terminal 1: Assinante (Simula o Dashboard recebendo dados)
mosquitto_sub -h localhost -t /inteli/esp32/sinal_wifi

# Terminal 2: Publicador (Simula o ESP32 enviando um dado)
mosquitto_pub -h localhost -t /inteli/esp32/sinal_wifi -m "-55"
```

📝 Documentação do Projeto IoT: Monitoramento de Sinal Wi-Fi (RSSI) com ESP32 e MQTT
1. 💡 Contexto da Atividade
Este projeto tem como objetivo principal monitorar a Potência do Sinal de Rádio (RSSI) de uma rede Wi-Fi específica, utilizando um microcontrolador ESP32, e publicar esses dados em tempo real em uma plataforma de visualização através do protocolo MQTT (Message Queuing Telemetry Transport).

A atividade culmina na realização de um experimento para simular o efeito da Gaiola de Faraday em um ambiente controlado (o elevador do Inteli), registrando a queda abrupta e posterior recuperação do sinal Wi-Fi (dBm) em um gráfico contínuo.

Shutterstock
Explorar

2. ⚙️ Uso e Configuração do Mosquitto Broker (Local)
O Mosquitto atua como o Broker MQTT, o intermediário que recebe as mensagens do ESP32 (Publisher) e as envia para o Dashboard Python (Subscriber).

Instalação e Execução (Local):

O Mosquitto deve ser instalado e iniciado como um serviço no seu computador, garantindo que ele esteja escutando na porta padrão 1883.

Instalação (Windows/macOS/Linux): Siga as instruções específicas para o seu sistema operacional na documentação oficial do Mosquitto.

Execução: Após a instalação, garanta que o serviço Mosquitto esteja rodando. O broker escutará as conexões de clientes (ESP32 e Dashboard) na porta 1883.

Teste de Conectividade:

Use as ferramentas de linha de comando do Mosquitto (mosquitto_sub e mosquitto_pub) para confirmar que o broker está funcionando.

Bash

# Terminal 1: Assinante (Simula o Dashboard recebendo dados)
mosquitto_sub -h localhost -t /inteli/esp32/sinal_wifi

# Terminal 2: Publicador (Simula o ESP32 enviando um dado)
mosquitto_pub -h localhost -t /inteli/esp32/sinal_wifi -m "-55"
3. 🤖 Código do ESP32 (C++ com POO e Ponteiros)
O código da ESP32 foi desenvolvido com Programação Orientada a Objetos (POO) e utiliza um ponteiro para a instância do cliente MQTT, encapsulando a lógica de conectividade e publicação.

``` jsx
#include <WiFi.h>
#include <PubSubClient.h> // Biblioteca para MQTT

// --- Configurações Estáticas ---
const char* WIFI_SSID = "SUA_REDE_WIFI";       
const char* WIFI_PASSWORD = "SUA_SENHA_WIFI";   
const char* MQTT_SERVER = "192.168.1.100";      // << AJUSTE ESTE IP (ou use o IP local do seu PC)
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_RSSI_Client_POO";
const char* MQTT_TOPIC_PUBLISH = "/inteli/esp32/sinal_wifi"; 
const long PUBLISH_INTERVAL_MS = 1000; 

WiFiClient espClient;

class RssiMqttClient {
private:
    // Ponteiro para o cliente MQTT
    PubSubClient* mqttClient; 
    long lastPublishTime = 0;
    
    // ... [Métodos de Conexão e Reconexão OMITIDOS para brevidade]

public:
    // Construtor: Aloca o objeto MQTT via ponteiro
    RssiMqttClient(WiFiClient& client) {
        mqttClient = new PubSubClient(client); 
        Serial.begin(115200);
    }
    
    // Destrutor: Libera a memória do ponteiro
    ~RssiMqttClient() {
        delete mqttClient;
        mqttClient = nullptr;
    }

    void begin() {
        // Lógica de connectWiFi() aqui
        // ...
        mqttClient->setServer(MQTT_SERVER, MQTT_PORT);
    }

    void handleLoop() {
        // Lógica de reconnectMQTT() e loop do MQTT aqui
        // ...
        
        long now = millis();
        if (now - lastPublishTime > PUBLISH_INTERVAL_MS) {
            lastPublishTime = now;
            publishRssi();
        }
    }
    
    void publishRssi() {
        long rssi_dbm = WiFi.RSSI();
        String payload = String(rssi_dbm);
        
        Serial.print("Potência (dBm): ");
        Serial.println(payload);

        // Uso do ponteiro -> para publicar
        if (mqttClient->publish(MQTT_TOPIC_PUBLISH, payload.c_str())) {
            Serial.println("MQTT Publicado.");
        } else {
            Serial.println("Falha na publicação.");
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

📝 Documentação do Projeto IoT: Monitoramento de Sinal Wi-Fi (RSSI) com ESP32 e MQTT
1. 💡 Contexto da Atividade
Este projeto tem como objetivo principal monitorar a Potência do Sinal de Rádio (RSSI) de uma rede Wi-Fi específica, utilizando um microcontrolador ESP32, e publicar esses dados em tempo real em uma plataforma de visualização através do protocolo MQTT (Message Queuing Telemetry Transport).

A atividade culmina na realização de um experimento para simular o efeito da Gaiola de Faraday em um ambiente controlado (o elevador do Inteli), registrando a queda abrupta e posterior recuperação do sinal Wi-Fi (dBm) em um gráfico contínuo.

Shutterstock
Explorar

2. ⚙️ Uso e Configuração do Mosquitto Broker (Local)
O Mosquitto atua como o Broker MQTT, o intermediário que recebe as mensagens do ESP32 (Publisher) e as envia para o Dashboard Python (Subscriber).

Instalação e Execução (Local):

O Mosquitto deve ser instalado e iniciado como um serviço no seu computador, garantindo que ele esteja escutando na porta padrão 1883.

Instalação (Windows/macOS/Linux): Siga as instruções específicas para o seu sistema operacional na documentação oficial do Mosquitto.

Execução: Após a instalação, garanta que o serviço Mosquitto esteja rodando. O broker escutará as conexões de clientes (ESP32 e Dashboard) na porta 1883.

Teste de Conectividade:

Use as ferramentas de linha de comando do Mosquitto (mosquitto_sub e mosquitto_pub) para confirmar que o broker está funcionando.

Bash

# Terminal 1: Assinante (Simula o Dashboard recebendo dados)
mosquitto_sub -h localhost -t /inteli/esp32/sinal_wifi

# Terminal 2: Publicador (Simula o ESP32 enviando um dado)
mosquitto_pub -h localhost -t /inteli/esp32/sinal_wifi -m "-55"
3. 🤖 Código do ESP32 (C++ com POO e Ponteiros)
O código da ESP32 foi desenvolvido com Programação Orientada a Objetos (POO) e utiliza um ponteiro para a instância do cliente MQTT, encapsulando a lógica de conectividade e publicação.

C++

#include <WiFi.h>
#include <PubSubClient.h> // Biblioteca para MQTT

// --- Configurações Estáticas ---
const char* WIFI_SSID = "SUA_REDE_WIFI";       
const char* WIFI_PASSWORD = "SUA_SENHA_WIFI";   
const char* MQTT_SERVER = "192.168.1.100";      // << AJUSTE ESTE IP (ou use o IP local do seu PC)
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_RSSI_Client_POO";
const char* MQTT_TOPIC_PUBLISH = "/inteli/esp32/sinal_wifi"; 
const long PUBLISH_INTERVAL_MS = 1000; 

WiFiClient espClient;

class RssiMqttClient {
private:
    // Ponteiro para o cliente MQTT
    PubSubClient* mqttClient; 
    long lastPublishTime = 0;
    
    // ... [Métodos de Conexão e Reconexão OMITIDOS para brevidade]

public:
    // Construtor: Aloca o objeto MQTT via ponteiro
    RssiMqttClient(WiFiClient& client) {
        mqttClient = new PubSubClient(client); 
        Serial.begin(115200);
    }
    
    // Destrutor: Libera a memória do ponteiro
    ~RssiMqttClient() {
        delete mqttClient;
        mqttClient = nullptr;
    }

    void begin() {
        // Lógica de connectWiFi() aqui
        // ...
        mqttClient->setServer(MQTT_SERVER, MQTT_PORT);
    }

    void handleLoop() {
        // Lógica de reconnectMQTT() e loop do MQTT aqui
        // ...
        
        long now = millis();
        if (now - lastPublishTime > PUBLISH_INTERVAL_MS) {
            lastPublishTime = now;
            publishRssi();
        }
    }
    
    void publishRssi() {
        long rssi_dbm = WiFi.RSSI();
        String payload = String(rssi_dbm);
        
        Serial.print("Potência (dBm): ");
        Serial.println(payload);

        // Uso do ponteiro -> para publicar
        if (mqttClient->publish(MQTT_TOPIC_PUBLISH, payload.c_str())) {
            Serial.println("MQTT Publicado.");
        } else {
            Serial.println("Falha na publicação.");
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
4. 📊 Código do Dashboard em Python (Plotly Dash)
O Dashboard usa as bibliotecas Paho-MQTT para subscrever os dados e Plotly Dash para criar a visualização do gráfico de série temporal, atualizando a cada 2 segundos. O MQTT_SERVER deve ser localhost se o Mosquitto estiver rodando localmente.

``` jsx

import dash
from dash import dcc
from dash import html
from dash.dependencies import Output, Input
import plotly.graph_objects as go
import pandas as pd
from collections import deque
import paho.mqtt.client as mqtt
import threading

# --- Configurações do Broker ---
MQTT_SERVER = "localhost" # Mosquitto está rodando localmente
MQTT_PORT = 1883
MQTT_TOPIC = "/inteli/esp32/sinal_wifi"

# Deques para armazenar dados em tempo real
MAX_DATA_POINTS = 300
data_time = deque(maxlen=MAX_DATA_POINTS)
data_rssi = deque(maxlen=MAX_DATA_POINTS)

# --- Funções de Callback MQTT (on_connect, on_message) e Thread de Loop OMITIDAS ---

# Inicia o cliente MQTT em uma thread
# mqtt_thread = threading.Thread(target=mqtt_loop)
# mqtt_thread.daemon = True
# mqtt_thread.start()

# --- Configuração do Dashboard (Plotly Dash) ---
app = dash.Dash(__name__)

app.layout = html.Div(
    children=[
        html.H1("📊 Monitoramento de Sinal WiFi (RSSI) - ESP32/MQTT"),
        dcc.Graph(id='live-rssi-graph'),
        dcc.Interval(
            id='interval-component',
            interval=2*1000, # Atualiza o gráfico a cada 2 segundos
            n_intervals=0
        ),
        html.Div(id='current-rssi-display')
    ]
)

# --- Callback para Atualização do Gráfico ---
@app.callback(
    [Output('live-rssi-graph', 'figure'),
     Output('current-rssi-display', 'children')],
    [Input('interval-component', 'n_intervals')]
)
def update_graph(n):
    # ... Lógica de criação do gráfico (Figura Plotly)
    
    current_rssi = data_rssi[-1] if data_rssi else "N/A"
    display_text = f"RSSI Atual: {current_rssi} dBm"

    return fig, display_text

if __name__ == '__main__':
    app.run_server(debug=True, host='0.0.0.0')

```

5. 🌡️ Matriz de Calor do Sinal (RSSI)
Esta seção descreve a observação do gráfico do Dashboard durante os cenários de teste. O valor RSSI é expresso em dBm (decibéis em relação a 1 miliwatt), onde valores mais próximos de 0 (e, portanto, menos negativos) indicam um sinal mais forte.

5.1. Teste Fora do Elevador (Ambiente Aberto)

| Condição | RSSI Médio (dBm) | Variação (dBm) | Testes Realizados | Observação |
| :---: | :---: | :---: | :---: | :---: |
| Próximo ao Roteador | $[-40 \text{ a } -50]$ | Baixa ($\pm 2$) | O dispositivo foi posicionado em uma mesa a poucos metros do ponto de acesso Wi-Fi. | **Forte Sinal:** O gráfico apresenta uma linha estável e alta (próxima de $-40 \text{ dBm}$). Pequenas flutuações são devidas a ruído ambiental normal. |
| Distante (Outra Sala) | $[-65 \text{ a } -75]$ | Média ($\pm 5$) | O dispositivo foi movido para uma sala separada por uma ou duas paredes. | **Sinal Moderado/Bom:** O valor de dBm diminuiu, mas permaneceu estável. |

Teste Dentro do Elevador (Simulação Gaiola de Faraday)

| Condição | Início (dBm) | Queda Mínima (dBm) | Recuperação (dBm) | Testes Realizados | Observação |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Entrada no Elevador** | $[-55]$ | $[-90 \text{ a } -100]$ | $[-55]$ | O ESP32 foi levado para dentro do elevador do Inteli. A porta foi fechada e mantida assim por 5 segundos. | **Bloqueio Efetivo:** Houve uma queda **drástica** e **imediata** no RSSI. O gráfico exibe um pico negativo acentuado, demonstrando que a estrutura metálica do elevador bloqueou a maior parte das ondas de rádio. |
| **Saída do Elevador** | $[-95]$ | N/A | $[-50 \text{ a } -60]$ | Após 5 segundos, a porta foi aberta e o ESP32 foi retirado do elevador. | **Recuperação Rápida:** O sinal recuperou-se quase instantaneamente, comprovando o efeito temporário do bloqueio e a capacidade do projeto de registrar a variação em tempo real. |


🎬 Descrição e Observações FinaisDescrição dos Testes:Os testes seguiram as etapas de Baseline (sinal forte), Teste de Distância (sinal moderado) e a Simulação da Gaiola de Faraday no elevador. O sistema ESP32/MQTT forneceu dados contínuos para o Dashboard em Python, permitindo a visualização imediata dos efeitos ambientais no sinal Wi-Fi.Observação Feita:A observação mais significativa foi a validação do princípio da Gaiola de Faraday. O gráfico da dashboard registrou claramente que, no momento exato em que a estrutura metálica do elevador foi fechada, o valor do RSSI despencou (movendo-se para perto de $-95\text{ dBm}$). A queda é um registro da atenuação do campo eletromagnético. A medição subiu de forma quase instantânea ao seu valor inicial assim que o dispositivo foi retirado do ambiente blindado, confirmando a capacidade do projeto de monitorar variações de rádio frequência em tempo real.