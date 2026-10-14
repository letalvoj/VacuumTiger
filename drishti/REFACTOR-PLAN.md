# REFACTOR-PLAN.md — Drishti (Análise + Proposta de Modularização)

> **Estado**: análise only — nada foi refatorado. UI de bancada não é urgente; este
> documento guarda o plano para quando houver janela de manutenção.
> Inventariado em 2026-10-14 sobre o commit `5e8d92e`.

---

## 1. Estado atual

### 1.1 Números

| Métrica | Valor |
|---|---|
| LOC Python total | 8.552 |
| LOC em bindings gerados (`*_pb2.py`) | ~404 (não editar à mão) |
| LOC de código real | ~8.150 |
| Qt Widgets | 24 classes |
| Threads de rede (QThread) | 3 + `RobotClient` legado |

### 1.2 Inventário arquivo → papel

```
drishti/
├── drishti_ui.py (159)            # entry point; subcomandos sensor|slam; argparse + QApplication
├── drishti.py (489)               # LEGADO (demo console msgpack) — RobotClient, _recv_exact, demo_side_brush
├── debug_proto.py (189)           # debug ad-hoc do DhruvaSLAM (scanner de mensagens)
├── proto/                         # bindings gerados (protobuf)
│   ├── sangamio_pb2.py            #   gerar: protoc --python_out=. -I../../sangam-io/proto ...
│   ├── dhruva_pb2.py
│   └── dhruva_slam/proto/dhruva_pb2.py  # ÓRFÃO — commit duplicado, ZERO referências; remover
└── ui/
    ├── main_window.py (447)       # [sensor] MainWindow: telemetria SangamIO + painel de controle
    ├── slam_main_window.py (383)  # [slam]   SlamMainWindow: mapa, nav, 16 .connect()
    ├── processors/
    │   └── imu_processor.py (203) # IMUProcessor — usado APENAS por robot_3d_view
    ├── threads/
    │   ├── telemetry_thread.py (443) # SangamIO: TCP 5555 comando + UDP 5556 sensores
    │   ├── odometry_thread.py (277)  # DhruvaSLAM: TCP 5557 odometria/diagnósticos
    │   └── slam_thread.py (621)      # DhruvaSLAM: TCP+UDP 5557 streams + comandos SLAM
    └── widgets/
        ├── robot_3d_view.py (1117)   # Robot3DView + BatteryBar — vista 3D OpenGL (pyqtgraph.gl)
        ├── robot_3d_geometry.py (596) # geometria wireframe pura (numpy, sem Qt!)
        ├── control_panel.py (976)    # sensor: Lidar/CliffIR/Dustbox/LED/Calibration/Power/DriveControl + ControlPanel
        ├── slam_control_panel.py (588) # slam: StatusDisplay/MappingProgress/MappingControls/MapListWidget
        ├── slam_view.py (488)        # mapa 2D pyqtgraph + trajetória
        ├── diagnostics_panel.py (331) # TimingBar/MatchQualityBar/StatRow
        ├── navigation_panel.py (314)
        ├── lidar_scan_overlay.py (266) # LidarScanWidget (2D) + LidarScanOverlay (overlays 2D)
        ├── compact_actuator.py (218)  # MiniActuator + CompactActuatorRow
        └── collapsible_group.py (125)
```

### 1.3 Acoplamento

Baixo acoplamento inter-módulo — imports cruzados verificáveis:

- `drishti_ui` → `ui/main_window`, `ui/slam_main_window`
- `main_window` → `ui.threads.telemetry_thread`, `ui.widgets.robot_3d_view`, `ui.widgets.control_panel`
- `slam_main_window` → `ui.threads.slam_thread`, `ui.widgets.slam_view`, `ui.widgets.slam_control_panel`, `ui.widgets.navigation_panel`
- `ui/widgets/control_panel` → `collapsible_group`, `compact_actuator`
- `ui/widgets/robot_3d_view` → `lidar_scan_overlay`, `robot_3d_geometry`, `ui.processors.imu_processor` **(ponto de acoplamento feio: widget importa processor de pacote irmão)**
- `ui/widgets/diagnostics_panel` → `collapsible_group`
- **Nenhum import entre os mundos sensor e slam** (exceto via drishti_ui).

Problema estrutural real: os 3 threads duplicam o mesmo `sys.path.insert(0, ...)`
para achar `proto/` — hack frágil que quebra se alguém importar `ui.threads` de
outro lugar (ex. testes).

### 1.4 Duplicação (maior concentração de débito)

| Duplicação | Onde | Esforço p/ extrair |
|---|---|---|
| `_recv_exact()` | `drishti.py`, `ui/threads/odometry_thread.py`, `ui/threads/slam_thread.py` (+ variante com timeout em `drishti.py`) | trivial |
| Framing length-prefix (`struct.unpack('>I', …)` + recv loop) | 6+ sites em `drishti.py`, `telemetry_thread`, `odometry_thread`, `slam_thread`, `debug_proto.py` | trivial |
| Loop de reconexão (`while self._running: try connect / emit / msleep(2000) / continue`) | idem em 3 threads | médio — comportamento ligeiramente distinto entre elas |
| protobuf → dict (construção de payload com dezenas de chaves) | `telemetry_thread` (8), `odometry_thread` (63), `slam_thread` (88) construtores de dict quase idênticos | maior — cada payload diverge em campos |
| `sys.path.insert` p/ achar `proto/` | 3 threads + `debug_proto.py` | trivial |
| Cores hex inline (~15 cores repetidas) | ~200 chamadas `setStyleSheet` espalhadas em 10 arquivos (67 em `control_panel` sozinho) | médio — migrar p/ módulo de tema |
| Boilerplate de janela (title bar, reconexão, status bar, keyboard handling) | `main_window` vs `slam_main_window` | médio |
| Pattern de comando (armazenar fila + emitir sinal + resposta) | 8 métodos de comando em `slam_thread` (start/stop_mapping, rename/delete map, etc.) | médio |

### 1.5 Bugs latentes (achados durante inventário, não corrigir agora)

- `requirements.txt` **não lista `msgpack`**, mas `drishti.py` importa — console client quebra num venv limpo.
- `drishti.py` default `--robot vacuum` vs README/UI/UI default `192.168.68.101` — inconsistency de conveniência, não crítico.
- `proto/dhruva_slam/proto/dhruva_pb2.py` órfão binding duplicado.

---

## 2. Pontos de dor

1. **Threads de rede são God-Object**: cada QThread faz socket/reconexão/framing/parse/emit num único arquivo de 620/443/277 linhas. Mudança no protocolo = tocar 3 arquivos.
2. **`control_panel.py` (976 LOC)**: 8 classes QWidget num único arquivo — cada uma pequena, mas o agrupamento fornece prova de que o arquivo como um todo impede navegação rápida.
3. **`robot_3d_view.py` (1.117 LOC)**: mistura: barra de bateria Qt, overlay Qt (IMU/botões/lidar), cena 3D OpenGL, e lógica de animação das rodas. Claramente 3 responsabilidades.
4. **Payloads protobuf→dict sem modelo único**: schemas não declarados em nenhum lugar; cada thread reconstrói dicionários à mão.
5. **Sem tema central**: cores hex espalhadas em 10 arquivos.
6. **Dois modos de janela sem base comum**: `MainWindow` e `SlamMainWindow` repetem keyboard E-Stop wiring, reconnect overlay e status bar.

---

## 3. Plano de extração faseado

> Regra geral: **uma fase = um commit revisável**, cada fase termina com os
> modos rodando idênticos ao modo atual (verificação por inspeção visual +
> packet-count comparado).

### Verificação padrão (aplicada em toda fase)

```bash
# 1. Sem erros de sintaxe/import
cd drishti && python -m compileall -q .
# 2. App abre offscreen (sem daemon) — não deve crashar
QT_QPA_PLATFORM=offscreen timeout 5 python drishti_ui.py sensor --robot localhost --port 10 || true
# 3. Teste behavior real: rodar sangam-io mock + sensor mode e inspecionar
#    contagem de pacotes por segundo no log
# 4. pytest (a partir da fase 3) — não existem testes hoje
```

### Fase 0 — Higiene (30 min)

- Adicionar `msgpack` ao `requirements.txt` (bug real: `drishti.py` importa mas não declara).
- Remover `proto/dhruva_slam/` (órfão).
- Ativar `python -m compileall` como smoke test + `pytest` em `drishti/tests/` (com 2-3 testes de framing).
- Configurar lint básico (`ruff`): desativar regras de Qt e manter só erros reais.

**Verificação**: app sensor + slam abrem; nenhuma feature muda.

### Fase 1 — `drishti/core/` (protocolo puro, sem Qt) (1-2h)

Extrair de `ui/threads/*` e `drishti.py` para:

```
drishti/core/
├── __init__.py
├── framing.py       # recv_exact(), read_message(), write_message() (length-prefix)
├── transport.py     # reconnecting TCP/UDP socket wrapper (backoff, log, stop flag)
└── codec.py         # protobuf→dict converters (ver Fase 1b)
```

- Migrar os 3 threads e `drishti.py` para importar de `core` (apagar as 4 cópias de `_recv_exact`).
- Matar os 3 `sys.path.insert` — `proto/` fica importável como pacote com `drishti/` na raiz (`from drishti.proto import sangamio_pb2`).
- **NÃO** alterar ordens de sinal emitidos nem formato de payload (mantém interface pública para `main_window`/`slam_main_window`).

**Verificação**: `pytest` do framing contra casos do `debug_proto.py` (bytes reais capturados em `debug_proto.py` servem de fixture); screenshots do app batem.

### Fase 1b — Decoders declarativos (opcional, maior)

- Em cada thread, declarar protobuf→dict como **spec declarativa** (dict de campo→chave)
  em vez de dicionário manual de 20-40 chaves.
- `slam_thread._process_stream_message` (241 linhas de ifs) vira tabela.
- Resultado: reduz `slam_thread` de 621 → ~300 LOC.

**Verificação**: testes unitários de codec com protobuf sintético; byte-equality do dict payload antes/depois (snapshot test).

### Fase 2 — Base Thread reutilizável (1h)

`core/protocol_thread.py::ProtocolThread(QThread)` com o loop run/stop/reconnect
padrão; 3 threads herdam e registram apenas handler de payload.

- Total thread LOC target: 621+443+277 = 1.341 → ~600.

**Verificação**: sensor mode contra `sangam-io` mock (simulated device); SLAM mode contra dhruva-slam em `--idle` (já existe subcomando); conferir que os sinais `connection_status` continuam a ser emitidos nos mesmos estágios.

### Fase 3 — Desmembrar `control_panel.py` e tema (2-3h)

```
ui/widgets/panels/sensor/
├── lidar.py, cliff_ir.py, dustbox.py, led.py,
├── calibration.py, power.py, drive.py        (extraídos 1:1)
ui/theme.py                                    # palette: cores hex das 15 cores únicas
```

- `control_panel.py` fica só com `ControlPanel` (o compositor) importando as widgets.
- Substituir cores hex literal por constantes do tema — max ~100 call sites, cada um 1:1.

**Verificação**: screenshot diff do painel sensor antes/depois (pode ser manual — 1 clique por grupo); `grep -c '#[0-9A-Fa-f]\{6\}' ui/widgets/panels/` deve cair a zero.

### Fase 4 — Split `robot_3d_view.py` + geometria testável (2-3h)

```
drishti/core/geometry.py     # robot_3d_geometry.py movido (puro numpy → pode ganhar pytest)
ui/widgets/robot_3d/
├── view.py                  # Robot3DView (cena 3D)
├── overlays/imu.py, battery.py, buttons.py, lidar.py
└── battery_bar.py
```

**Verificação**: visual no app + `pytest core/geometry.py` (funções são determinísticas).

### Fase 5 — Janela base + limpeza (1h)

- `ui/base_window.py` com: title bar, status bar, teclado E-Stop, overlay de reconexão — compartilhamento entre `MainWindow` e `SlamMainWindow`.
- Mover `drishti.py` (legado) → `tools/legacy_client.py` + nota de deprecated; ajustar `README`.
- Resultado final: LOC de drishti ~7.000, ~7.000 real, com menores unidades de teste.

**Verificação**: rodar os dois modos end-to-end contra SangamIO mock + DhruvaSLAM real.

---

## 4. O que NÃO mexer

| Item | Por quê |
|---|---|
| **Protocolo de rede** (ports 5555/5556/5557, framing, topics, ordem dos campos de payload) | contrato com SangamIO daemon (Rust, em produção no robô). UI é descartável; protocolo não. |
| **`proto/*.py` bindings gerados** | só regenerar via `protoc`, nunca editar a mão |
| **Comportamento de E-Stop / drive keyboard** | segurança física da bancada — zero risco tolerado |
| **`drishti.png`, assets** | referenciado no README |
| **`ui/processors/imu_processor.py`** | já está isolado e seu comportamento (roll/pitch/yaw na vista 3D) é calibrado visualmente — mexer exige verificação visual robusta |
| **Offsets de câmera / preset names da vista 3D** | valor de calibração manual do operador da bancada |
| **`debug_proto.py`** | ferramenta de diagnóstico que funciona; não vale o esforço |

---

## 5. Métrica de sucesso (quando executado)

- [ ] Nenhum arquivo Python com >500 LOC
- [ ] Zero duplicação de `_recv_exact`/framing (1 fonte)
- [ ] Threads sem conhecimento de sockets raw (usar `core/transport`)
- [ ] `pytest core/` cobre framing + codecs + geometry (≥ 3 testes)
- [ ] Dois modos UI rodando visualmente idêntico ao estado atual
- [ ] `drishti.py` legado sobrevive (movido, não deletado)
