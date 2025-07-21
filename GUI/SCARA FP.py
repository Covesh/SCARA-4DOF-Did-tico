import sys
import serial
import time
import json
from pathlib import Path
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                 QPushButton, QLabel, QGridLayout, QFrame, QLineEdit, QTabWidget,
                                 QSlider, QStatusBar, QScrollArea, QCheckBox, QFileDialog)
from PySide6.QtCore import QThread, QObject, Signal, Slot, Qt, QTimer, QFile
from PySide6.QtGui import QIcon

# --- CONFIGURAÇÕES ---
ARDUINO_PORT = "COM14"  # VerifiCA a porta serial do Arduino
BAUDRATE = 9600
SEQUENCE_FILE = Path("sequences.json")  # Ficheiro para guardar automaticamente as sequências
MAX_SEQUENCE_POSITIONS = 10
JOG_INTERVAL_MS = 150
JOG_STEP_XY = 2.0
JOG_STEP_Z = 1.0
JOG_STEP_U = 2.0

# --- Worker para Comunicação Serial ---
# Esta classe corre numa thread separada para não bloquear a interface gráfica enquanto espera por dados da porta serial.
class SerialWorker(QObject):
    data_received = Signal(str)
    connection_failed = Signal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port, self.baudrate = port, baudrate
        self.is_running = True
        self.ser = None

    @Slot()
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2) # Espera para a conexão serial estabilizar
            self.data_received.emit("MSG:Conexão estabelecida.")
            while self.is_running:
                if self.ser and self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line: 
                            self.data_received.emit(line)
                    except UnicodeDecodeError: 
                        # Ignora erros de decodificação que podem acontecer
                        pass
                time.sleep(0.05)
            if self.ser: self.ser.close()
        except serial.SerialException as e:
            self.connection_failed.emit(str(e))

    def stop(self):
        self.is_running = False

# --- Classe Principal da Interface Gráfica (HMI) ---
class HMI_Completa(QMainWindow):
    # Constantes para mapeamento da garra de ângulo para milímetros
    GRIPPER_ANGLE_OPEN = 45
    GRIPPER_ANGLE_CLOSE = 175
    GRIPPER_MM_OPEN = 32.0
    GRIPPER_MM_CLOSE = 1.5

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HMI SCARA - TA")
        self.setGeometry(100, 100, 950, 850)

        # Variáveis de estado para controlar o robô e a interface
        self.motors_on = False
        self.is_running_sequence = False
        self.graceful_stop_requested = False
        self.robot_current_wpos = [0.0] * 4
        self.robot_current_jpos = [0.0] * 4
        self.robot_current_gripper_angle = self.GRIPPER_ANGLE_OPEN
        self.sequence_to_run = []
        self.current_sequence_step = 0
        self.running_seq_num = -1
        
        self.load_sequences() # Carrega as sequências do ficheiro JSON
        self.setup_ui()       # Constrói todos os widgets da interface
        self.setup_connections() # Liga os botões às suas funções
        self.setup_serial()      # Inicia a comunicação com o Arduino
        self.update_gripper_ui(self.robot_current_gripper_angle) # Garante que a UI da garra está correta no arranque

    def setup_ui(self):
        """Cria a estrutura principal da interface gráfica."""
        main_container = QWidget()
        self.setCentralWidget(main_container)
        container_layout = QVBoxLayout(main_container)
        container_layout.setContentsMargins(0,0,0,0)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        container_layout.addWidget(scroll_area)
        content_widget = QWidget()
        scroll_area.setWidget(content_widget)
        self.main_layout = QVBoxLayout(content_widget)
        self.main_layout.setSpacing(10)
        
        # Criação dos diferentes painéis da UI
        self.create_top_control_panel()
        self.create_tabs()
        self.create_gripper_panel() 
        self.create_position_panel()
        self.create_sequence_panel()
        self.main_layout.addStretch()
        self.create_status_bar()

    def create_top_control_panel(self):
        """Cria os botões de controlo principais (Calibrar, Home, Stop, etc.)."""
        top_frame = QFrame()
        top_layout = QHBoxLayout(top_frame)
        
        self.btn_autocal = QPushButton("Calibrar")
        self.btn_autocal.setStyleSheet("background-color: #D08770; color: white;")

        self.btn_gohome = QPushButton("Home")
        self.btn_gohome.setStyleSheet("background-color: #A3BE8C;")

        self.btn_toggle_motors = QPushButton("⏻")
        self.btn_toggle_motors.setToolTip("Motores Desligados")
        
        self.btn_estop = QPushButton("STOP")
        self.btn_estop.setToolTip("PARAGEM DE EMERGÊNCIA")
        self.btn_estop.setStyleSheet("background-color: #BF616A; color: white; font-weight: bold;")
        
        top_layout.addWidget(self.btn_autocal)
        top_layout.addWidget(self.btn_gohome)
        top_layout.addStretch()
        top_layout.addWidget(self.btn_toggle_motors)
        top_layout.addWidget(self.btn_estop)
        
        self.main_layout.addWidget(top_frame)

    def create_tabs(self):
        """Cria as abas para controlo por IK (Jogging) e FK (Juntas)."""
        self.tabs = QTabWidget()
        self.main_layout.addWidget(self.tabs)
        ik_tab = QWidget()
        self.populate_ik_tab(QVBoxLayout(ik_tab))
        self.tabs.addTab(ik_tab, "Controlo IK (Jogging)")
        fk_tab = QWidget()
        self.populate_fk_tab(QVBoxLayout(fk_tab))
        self.tabs.addTab(fk_tab, "Controlo FK (Juntas Individuais)")

    def populate_ik_tab(self, layout):
        """Preenche a aba de controlo por cinemática inversa (IK)."""
        goto_frame = QFrame()
        goto_layout = QHBoxLayout(goto_frame)
        goto_layout.addWidget(QLabel("<b>Ir para (X,Y,Z,U):</b>"))
        self.le_x, self.le_y, self.le_z, self.le_u = QLineEdit("200.0"), QLineEdit("0.0"), QLineEdit("150.0"), QLineEdit("0.0")
        self.btn_send_goto = QPushButton("Enviar")
        for w in [QLabel("X:"), self.le_x, QLabel("Y:"), self.le_y, QLabel("Z:"), self.le_z, QLabel("U:"), self.le_u, self.btn_send_goto]:
            goto_layout.addWidget(w)
        layout.addWidget(goto_frame)
        
        jog_frame = QFrame()
        jog_grid = QGridLayout(jog_frame)
        self.jog_buttons = {
            "U,-1": self.create_jog_button("U,-1", "-U"), "Y,1":  self.create_jog_button("Y,1", "+Y"),
            "U,1":  self.create_jog_button("U,1", "+U"), "X,-1": self.create_jog_button("X,-1", "-X"),
            "Y,-1": self.create_jog_button("Y,-1", "-Y"), "X,1":  self.create_jog_button("X,1", "+X"),
            "Z,1":  self.create_jog_button("Z,1", "+Z"), "Z,-1": self.create_jog_button("Z,-1", "-Z")
        }
        jog_grid.addWidget(self.jog_buttons["U,-1"], 0, 0); jog_grid.addWidget(self.jog_buttons["Y,1"], 0, 1); jog_grid.addWidget(self.jog_buttons["U,1"], 0, 2)
        jog_grid.addWidget(self.jog_buttons["X,-1"], 1, 0); jog_grid.addWidget(self.jog_buttons["Y,-1"], 1, 1); jog_grid.addWidget(self.jog_buttons["X,1"], 1, 2)
        z_layout = QVBoxLayout(); z_layout.addWidget(self.jog_buttons["Z,1"]); z_layout.addWidget(self.jog_buttons["Z,-1"])
        jog_grid.addLayout(z_layout, 0, 4, 2, 1)
        
        speed_layout = QHBoxLayout()
        self.btn_speed_down, self.lbl_speed, self.btn_speed_up = QPushButton("<< Vel"), QLabel("Vel: 100%"), QPushButton("Vel >>")
        self.lbl_speed.setAlignment(Qt.AlignCenter)
        for w in [self.btn_speed_down, self.lbl_speed, self.btn_speed_up]: speed_layout.addWidget(w)
        jog_grid.addLayout(speed_layout, 2, 0, 1, 5)
        layout.addWidget(jog_frame)

    def create_jog_button(self, command, text):
        btn = QPushButton(text)
        btn.setProperty("jog_command", command)
        return btn

    def populate_fk_tab(self, layout):
        """Preenche a aba de controlo por cinemática direta (FK)."""
        fk_grid = QGridLayout()
        self.sliders = {
            "J1": self.create_joint_slider(0, 1800), 
            "J2": self.create_joint_slider(0, 1700),
            "J3": self.create_joint_slider(-1350, 1350), 
            "J4": self.create_joint_slider(-1350, 1350)
        }
        labels = ["J1 (Base)", "J2 (Altura)", "J3 (Cotovelo)", "J4 (Garra)"]
        keys = ["J1", "J2", "J3", "J4"]
        for i, key in enumerate(keys):
            fk_grid.addWidget(QLabel(f"<b>{labels[i]}</b>"), i, 0)
            fk_grid.addWidget(self.sliders[key]["slider"], i, 1)
            fk_grid.addWidget(self.sliders[key]["label"], i, 2)
        layout.addLayout(fk_grid)
        layout.addStretch()

    def create_joint_slider(self, min_val, max_val):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        label = QLabel(f"{slider.value() / 10.0:.1f}")
        return {"slider": slider, "label": label}

    def create_position_panel(self):
        """Cria o painel que mostra as posições atuais do robô (cartesianas e de juntas)."""
        pos_frame = QFrame()
        pos_layout = QGridLayout(pos_frame)
        pos_layout.addWidget(QLabel("<b>Posição Atual</b>"), 0, 0, 1, 4)
        self.lbl_x_pos, self.lbl_y_pos, self.lbl_z_pos, self.lbl_u_pos = QLabel("X: ---"), QLabel("Y: ---"), QLabel("Z: ---"), QLabel("U: ---")
        self.lbl_j1_pos, self.lbl_j2_pos, self.lbl_j3_pos, self.lbl_j4_pos = QLabel("J1: ---"), QLabel("J2: ---"), QLabel("J3: ---"), QLabel("J4: ---")
        
        pos_layout.addWidget(self.lbl_x_pos, 1, 0); pos_layout.addWidget(self.lbl_y_pos, 1, 1); pos_layout.addWidget(self.lbl_z_pos, 1, 2); pos_layout.addWidget(self.lbl_u_pos, 1, 3)
        pos_layout.addWidget(self.lbl_j1_pos, 2, 0); pos_layout.addWidget(self.lbl_j2_pos, 2, 1); pos_layout.addWidget(self.lbl_j3_pos, 2, 2); pos_layout.addWidget(self.lbl_j4_pos, 2, 3)
        self.main_layout.addWidget(pos_frame)

    def create_gripper_panel(self):
        """Cria o painel de controlo dedicado para a garra."""
        gripper_frame = QFrame()
        gripper_layout = QVBoxLayout(gripper_frame)

        title_label = QLabel("Controlo da Garra")
        title_label.setStyleSheet("font-size: 13pt; font-weight: bold; color: #D08770; padding-bottom: 5px;")
        title_label.setAlignment(Qt.AlignCenter)
        
        slider_layout = QHBoxLayout()
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(int(self.GRIPPER_MM_CLOSE * 10), int(self.GRIPPER_MM_OPEN * 10))
        slider_layout.addStretch(1)
        slider_layout.addWidget(self.gripper_slider, 2)
        slider_layout.addStretch(1)

        pos_display_layout = QHBoxLayout()
        pos_display_layout.addStretch()
        pos_display_layout.addWidget(QLabel("<b>Abertura da Garra:</b>"))
        self.lbl_gripper_pos = QLabel(f"--- mm")
        self.lbl_gripper_pos.setStyleSheet("font-size: 14pt; font-weight: bold; color: #EBCB8B; padding: 5px;")
        pos_display_layout.addWidget(self.lbl_gripper_pos)
        pos_display_layout.addStretch()

        button_layout = QHBoxLayout()
        button_layout.addStretch()
        self.btn_gripper_open = QPushButton("Abrir Garra")
        self.btn_gripper_close = QPushButton("Fechar Garra")
        button_layout.addWidget(self.btn_gripper_open)
        button_layout.addWidget(self.btn_gripper_close)
        button_layout.addStretch()

        gripper_layout.addWidget(title_label)
        gripper_layout.addLayout(slider_layout)
        gripper_layout.addLayout(pos_display_layout)
        gripper_layout.addLayout(button_layout)
        
        self.main_layout.addWidget(gripper_frame)

    def create_status_bar(self):
        """Cria a barra de status na parte inferior da janela."""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("Aguardando conexão com o robô...")
        self.status_bar.addWidget(self.status_label)
        self.show_status_message("Aguardando conexão com o robô...", "warning")

    def create_sequence_panel(self):
        """Cria toda a secção de gestão de sequências, incluindo os botões de I/O."""
        seq_frame = QFrame()
        seq_layout = QVBoxLayout(seq_frame)

        title_layout = QHBoxLayout()
        title_label = QLabel("<h2>Gestor de Sequências</h2>")
        self.btn_export = QPushButton("Guardar")
        self.btn_import = QPushButton("Abrir")
        self.btn_export.setToolTip("Guardar todas as sequências num ficheiro .txt")
        self.btn_import.setToolTip("Carregar sequências de um ficheiro .txt")
        
        title_layout.addWidget(title_label)
        title_layout.addStretch()
        title_layout.addWidget(self.btn_export)
        title_layout.addWidget(self.btn_import)
        seq_layout.addLayout(title_layout)

        self.seq_widgets = {}
        for i in range(1, 4):
            self.seq_widgets[i] = self.create_single_seq_ui(i)
            seq_layout.addWidget(self.seq_widgets[i]["main_frame"])
            self.update_seq_ui(i)
        self.main_layout.addWidget(seq_frame)

    def create_single_seq_ui(self, seq_num):
        """Cria a interface para uma única linha de sequência."""
        main_f = QFrame()
        main_f.setObjectName(f"seq_frame_{seq_num}")
        main_l, top_l = QVBoxLayout(main_f), QHBoxLayout()
        btn_toggle, btn_run, btn_stop, chk_loop = QPushButton(f"Editar Sequência {seq_num}"), QPushButton("▶ Correr"), QPushButton("■ Parar"), QCheckBox("Loop")
        btn_run.setStyleSheet("background-color: #A3BE8C;")
        btn_stop.setStyleSheet("background-color: #EBCB8B;")
        top_l.addWidget(btn_toggle); top_l.addStretch(); top_l.addWidget(chk_loop); top_l.addWidget(btn_run); top_l.addWidget(btn_stop)
        main_l.addLayout(top_l)
        scroll_area = QScrollArea(); scroll_area.setWidgetResizable(True); scroll_area.setVisible(False)
        positions_panel = QWidget(); pos_layout = QGridLayout(positions_panel); scroll_area.setWidget(positions_panel)
        widgets = {"main_frame": main_f, "toggle_btn": btn_toggle, "run_btn": btn_run, "stop_btn": btn_stop, "loop_chk": chk_loop, "scroll_area": scroll_area, "slots": []}
        
        headers = ["<b>Posição (X,Y,Z,U)</b>", "<b>Garra (mm)</b>", "<b>Delay(s)</b>", "<b>Vel(%)</b>"]
        for i, h in enumerate(headers): pos_layout.addWidget(QLabel(h), 0, i + 1)
        
        for i in range(MAX_SEQUENCE_POSITIONS):
            pos_lbl, gripper_lbl, delay_in, speed_in = QLabel("--"), QLabel("--"), QLineEdit("1.0"), QLineEdit("100")
            delay_in.setFixedWidth(50); speed_in.setFixedWidth(50)
            btn_save, btn_del = QPushButton("Gravar Posição"), QPushButton("X")
            
            row_widgets = [QLabel(f"Passo {i+1}:"), pos_lbl, gripper_lbl, delay_in, speed_in, btn_save, btn_del]
            for j, w in enumerate(row_widgets): pos_layout.addWidget(w, i + 1, j)
            
            widgets["slots"].append({"label": pos_lbl, "gripper_label": gripper_lbl, "delay_input": delay_in, "speed_input": speed_in, "save_btn": btn_save, "del_btn": btn_del})
        
        edit_bottom_layout = QHBoxLayout()
        btn_save_all, btn_clear = QPushButton("Salvar e Fechar"), QPushButton("Apagar Tudo")
        edit_bottom_layout.addStretch(); edit_bottom_layout.addWidget(btn_clear); edit_bottom_layout.addWidget(btn_save_all)
        
        pos_layout.addLayout(edit_bottom_layout, MAX_SEQUENCE_POSITIONS + 1, 0, 1, 7) 
        
        main_l.addWidget(scroll_area)
        widgets["save_all_btn"], widgets["clear_btn"] = btn_save_all, btn_clear
        return widgets

    def update_seq_ui(self, seq_num):
        """Atualiza os campos de uma linha de sequência com os dados guardados."""
        widgets = self.seq_widgets[seq_num]
        for i, slot in enumerate(widgets["slots"]):
            pos_data = self.sequences[seq_num]["positions"][i]
            if pos_data and isinstance(pos_data, dict):
                slot["label"].setText(f"X:{pos_data['pos'][0]:.1f}, Y:{pos_data['pos'][1]:.1f}, Z:{pos_data['pos'][2]:.1f}, U:{pos_data['pos'][3]:.1f}")
                slot["gripper_label"].setText(f"{pos_data.get('gripper', 0.0):.1f}")
                slot["delay_input"].setText(str(pos_data.get('delay', 1.0)))
                slot["speed_input"].setText(str(int(pos_data.get('speed', 1.0) * 100)))
            else:
                slot["label"].setText("--")
                slot["gripper_label"].setText("--")
                slot["delay_input"].setText("1.0")
                slot["speed_input"].setText("100")

    def toggle_sequence_panel(self, seq_num):
        """Mostra ou esconde o painel de edição de uma sequência."""
        scroll_area = self.seq_widgets[seq_num]["scroll_area"]
        scroll_area.setVisible(not scroll_area.isVisible())

    def save_pos_to_slot(self, s, p):
        """Guarda a posição atual do robô e da garra num passo da sequência."""
        widgets = self.seq_widgets[s]["slots"][p]
        try:
            delay = float(widgets["delay_input"].text())
            speed_pct = int(widgets["speed_input"].text())
            if not (50 <= speed_pct <= 150): raise ValueError("Velocidade entre 50% e 150%")
            gripper_mm = float(self.lbl_gripper_pos.text().replace(" mm", ""))
        except (ValueError, IndexError) as e:
            self.show_status_message(f"ERRO: Dados inválidos - {e}", "error")
            return
        
        self.sequences[s]["positions"][p] = { 
            "pos": [round(c, 2) for c in self.robot_current_wpos], 
            "gripper": gripper_mm,
            "delay": delay, 
            "speed": speed_pct / 100.0 
        }
        self.update_seq_ui(s)

    def del_pos_from_slot(self, s, p):
        """Apaga um passo de uma sequência."""
        self.sequences[s]["positions"][p] = None
        self.update_seq_ui(s)

    def save_sequences(self, seq_num_to_close):
        """Guarda todas as sequências no ficheiro JSON de auto-save."""
        for s_num, s_widgets in self.seq_widgets.items():
            for i, slot in enumerate(s_widgets["slots"]):
                if self.sequences[s_num]["positions"][i] is not None:
                    try:
                        self.sequences[s_num]["positions"][i]['delay'] = float(slot["delay_input"].text())
                        speed_pct = int(slot["speed_input"].text())
                        self.sequences[s_num]["positions"][i]['speed'] = (max(50, min(speed_pct, 150))) / 100.0
                    except (ValueError, KeyError): continue
        with open(SEQUENCE_FILE, 'w', encoding='utf-8') as f:
            json.dump(self.sequences, f, indent=4)
        self.show_status_message("Sequências salvas.", "success")
        if seq_num_to_close:
            self.seq_widgets[seq_num_to_close]["scroll_area"].setVisible(False)
            self.update_seq_ui(seq_num_to_close)

    def clear_sequence(self, s):
        """Limpa todos os passos de uma sequência."""
        self.sequences[s]["positions"] = [None] * MAX_SEQUENCE_POSITIONS
        self.save_sequences(None)
        self.update_seq_ui(s)

    def run_sequence(self, s):
        """Inicia a execução de uma sequência."""
        if self.is_running_sequence:
            self.show_status_message("ERRO: Sequência já em execução.", "error")
            return
        self.sequence_to_run = [step for step in self.sequences[s]["positions"] if step is not None]
        if not self.sequence_to_run:
            self.show_status_message("Aviso: Sequência vazia.", "warning")
            return
        self.graceful_stop_requested = False
        self.is_running_sequence = True
        self.current_sequence_step = 0
        self.running_seq_num = s
        self.run_next_seq_step()

    def stop_sequence(self):
        """Para a execução da sequência imediatamente."""
        if not self.is_running_sequence: return
        self.is_running_sequence = False
        self.running_seq_num = -1
        self.send_command("E_STOP\n")
        self.show_status_message("Paragem de emergência acionada!", "error")
        QTimer.singleShot(100, lambda: self.send_command("SET_SPEED_MULT:1.0\n"))

    def request_graceful_stop(self):
        """Pede para parar a sequência no final do passo atual."""
        if self.is_running_sequence:
            self.graceful_stop_requested = True
            self.show_status_message("A terminar o passo atual para parar...", "warning")

    def run_next_seq_step(self):
        """Executa o próximo passo da sequência ativa."""
        if not self.is_running_sequence: return
        loop_is_checked = self.seq_widgets[self.running_seq_num]["loop_chk"].isChecked()
        if self.current_sequence_step >= len(self.sequence_to_run):
            if loop_is_checked:
                self.current_sequence_step = 0
            else:
                self.is_running_sequence = False
                self.running_seq_num = -1
                self.show_status_message("Sequência concluída.", "success")
                self.send_command("SET_SPEED_MULT:1.0\n")
                return
        step_data = self.sequence_to_run[self.current_sequence_step]
        pos, speed = step_data['pos'], step_data.get('speed', 1.0)
        self.show_status_message(f"Passo {self.current_sequence_step + 1}: Movimentando para X:{pos[0]}, Y:{pos[1]}, Z:{pos[2]}, U:{pos[3]}...", "info")
        self.send_command(f"SET_SPEED_MULT:{speed:.2f}\n")
        QTimer.singleShot(50, lambda: self.send_command(f"GOTO_IK:{pos[0]},{pos[1]},{pos[2]},{pos[3]}\n"))

    def load_sequences(self):
        """Carrega as sequências do ficheiro JSON no arranque."""
        self.sequences = {}
        if SEQUENCE_FILE.exists():
            try:
                with open(SEQUENCE_FILE, 'r', encoding='utf-8') as f:
                    self.sequences = {int(k): v for k, v in json.load(f).items()}
            except (json.JSONDecodeError, ValueError): self.sequences = {}
        # Garante que a estrutura de dados está completa
        for i in range(1, 4):
            if i not in self.sequences:
                self.sequences[i] = {"name": f"Sequência {i}", "positions": [None] * MAX_SEQUENCE_POSITIONS}
            else:
                for j, pos in enumerate(self.sequences[i]["positions"]):
                    if pos and 'gripper' not in pos:
                        self.sequences[i]["positions"][j]['gripper'] = self.GRIPPER_MM_OPEN
                
                if len(self.sequences[i]["positions"]) < MAX_SEQUENCE_POSITIONS:
                    self.sequences[i]["positions"].extend([None] * (MAX_SEQUENCE_POSITIONS - len(self.sequences[i]["positions"])))

    def setup_connections(self):
        """Liga todos os sinais (ex: cliques de botão) às suas funções (slots)."""
        self.btn_autocal.clicked.connect(lambda: self.send_command("AUTO_CAL\n"))
        self.btn_gohome.clicked.connect(lambda: self.send_command("GOTO_HOME\n"))
        self.btn_toggle_motors.clicked.connect(self.toggle_motors)
        self.btn_estop.clicked.connect(self.stop_sequence)
        self.btn_send_goto.clicked.connect(self.send_manual_goto_command)
        self.btn_speed_down.clicked.connect(lambda: self.change_speed(-0.05))
        self.btn_speed_up.clicked.connect(lambda: self.change_speed(0.05))
        
        self.btn_gripper_open.clicked.connect(self.on_gripper_open_clicked)
        self.btn_gripper_close.clicked.connect(self.on_gripper_close_clicked)
        self.gripper_slider.valueChanged.connect(self.on_gripper_slider_moved)
        self.gripper_slider.sliderReleased.connect(self.on_gripper_slider_released)

        self.btn_import.clicked.connect(self.import_sequences)
        self.btn_export.clicked.connect(self.export_sequences)

        for btn in self.jog_buttons.values():
            btn.pressed.connect(self.on_jog_button_pressed)
            btn.released.connect(self.on_jog_button_released)
        for s_dict in self.sliders.values():
            s_dict["slider"].valueChanged.connect(lambda val, lbl=s_dict["label"]: lbl.setText(f"{val / 10.0:.1f}"))
            s_dict["slider"].sliderReleased.connect(self.on_slider_released)
        for i in range(1, 4):
            w = self.seq_widgets[i]
            w["toggle_btn"].clicked.connect(lambda c=False, s=i: self.toggle_sequence_panel(s))
            w["save_all_btn"].clicked.connect(lambda c=False, s=i: self.save_sequences(s))
            w["clear_btn"].clicked.connect(lambda c=False, s=i: self.clear_sequence(s))
            w["run_btn"].clicked.connect(lambda c=False, s=i: self.run_sequence(s))
            w["stop_btn"].clicked.connect(self.request_graceful_stop)
            for j, slot in enumerate(w["slots"]):
                slot["save_btn"].clicked.connect(lambda c=False, s_n=i, p_i=j: self.save_pos_to_slot(s_n, p_i))
                slot["del_btn"].clicked.connect(lambda c=False, s_n=i, p_i=j: self.del_pos_from_slot(s_n, p_i))
        self.jog_timer = QTimer(self)
        self.jog_timer.setInterval(JOG_INTERVAL_MS)
        self.jog_timer.timeout.connect(self.execute_jog_step)
        self.tabs.currentChanged.connect(self.on_tab_changed)

    def setup_serial(self):
        """Inicia a thread para comunicação serial."""
        self.thread = QThread()
        self.worker = SerialWorker(ARDUINO_PORT, BAUDRATE)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.data_received.connect(self.handle_serial_data)
        self.worker.connection_failed.connect(lambda err: self.show_status_message(f"FALHA NA CONEXÃO: {err}", "error"))
        self.thread.start()

    def send_command(self, cmd):
        """Envia um comando para o Arduino."""
        if self.worker and self.worker.ser and self.worker.ser.is_open:
            if "GOTO_IK" not in cmd:
                print(f"Comando enviado: {cmd.strip()}")
            self.worker.ser.write(cmd.encode('utf-8'))
        else:
            self.show_status_message("Erro: Conexão serial não está aberta.", "error")

    @Slot()
    def send_manual_goto_command(self):
        """Envia um comando de movimento para uma posição cartesiana específica."""
        try:
            coords = [float(w.text()) for w in [self.le_x, self.le_y, self.le_z, self.le_u]]
        except ValueError:
            self.show_status_message("ERRO: Coordenadas de input inválidas.", "error")
            return
        x,y,z,u = coords
        self.show_status_message(f"Movimentando para X:{x}, Y:{y}, Z:{z}, U:{u}...", "info")
        self.send_command(f"GOTO_IK:{x},{y},{z},{u}\n")

    def map_angle_to_mm(self, angle):
        """Converte o ângulo do servo (graus) para a abertura da garra (mm)."""
        angle_range = self.GRIPPER_ANGLE_CLOSE - self.GRIPPER_ANGLE_OPEN
        mm_range = self.GRIPPER_MM_CLOSE - self.GRIPPER_MM_OPEN
        if angle_range == 0: return self.GRIPPER_MM_OPEN
        dist_mm = self.GRIPPER_MM_OPEN + ((angle - self.GRIPPER_ANGLE_OPEN) * mm_range) / angle_range
        return max(min(dist_mm, self.GRIPPER_MM_OPEN), self.GRIPPER_MM_CLOSE)

    def map_mm_to_angle(self, mm):
        """Converte a abertura da garra (mm) para o ângulo do servo (graus)."""
        mm_range = self.GRIPPER_MM_OPEN - self.GRIPPER_MM_CLOSE
        angle_range = self.GRIPPER_ANGLE_OPEN - self.GRIPPER_ANGLE_CLOSE
        if mm_range == 0: return self.GRIPPER_ANGLE_CLOSE
        angle = self.GRIPPER_ANGLE_CLOSE + ((mm - self.GRIPPER_MM_CLOSE) * angle_range) / mm_range
        return int(max(min(angle, self.GRIPPER_ANGLE_CLOSE), self.GRIPPER_ANGLE_OPEN))

    def update_gripper_ui(self, angle):
        """Sincroniza todos os controlos da garra (slider e texto) com um novo valor de ângulo."""
        self.gripper_slider.blockSignals(True)
        self.robot_current_gripper_angle = angle
        gripper_mm = self.map_angle_to_mm(angle)
        self.lbl_gripper_pos.setText(f"{gripper_mm:.1f} mm")
        self.gripper_slider.setValue(int(gripper_mm * 10))
        self.gripper_slider.blockSignals(False)

    @Slot(str)
    def handle_serial_data(self, data):
        """Processa todas as mensagens recebidas do Arduino."""
        if not data.startswith(("WPOS", "POS", "GRIPPER")):
            message = self.translate_arduino_message(data)
            self.show_status_message(message["text"], message["level"])

        try:
            if ':' in data:
                cmd, val = data.split(':', 1)
                if "ERR" in cmd or "UNREACHABLE" in val or "PARAGEM" in val:
                    if self.is_running_sequence: self.stop_sequence()
                    
                elif cmd == "MSG":
                    if "Movimento concluido" in val and self.is_running_sequence:
                        if self.graceful_stop_requested:
                            self.is_running_sequence = False; self.running_seq_num = -1; self.graceful_stop_requested = False
                            self.show_status_message("Sequência parada com segurança.", "success")
                            self.send_command("SET_SPEED_MULT:1.0\n")
                            return
                        
                        step_data = self.sequence_to_run[self.current_sequence_step]
                        gripper_mm = step_data.get('gripper', self.GRIPPER_MM_OPEN)
                        angle_to_set = self.map_mm_to_angle(gripper_mm)
                        self.send_command(f"GRIPPER_SET:{angle_to_set}\n")
                        
                        delay = step_data.get('delay', 0)
                        
                        self.current_sequence_step += 1
                        QTimer.singleShot(int(delay * 1000), self.run_next_seq_step)

                    elif "Motores LIGADOS" in val: self.update_motors_button_ui(True)
                    elif "Motores DESLIGADOS" in val: self.update_motors_button_ui(False)
                elif cmd == "WPOS":
                    coords = [float(c) for c in val.split(',')]
                    self.robot_current_wpos = coords
                    self.lbl_x_pos.setText(f"X: {coords[0]:.2f}mm"); self.lbl_y_pos.setText(f"Y: {coords[1]:.2f}mm")
                    self.lbl_z_pos.setText(f"Z: {coords[2]:.2f}mm"); self.lbl_u_pos.setText(f"U: {coords[3]:.2f}°")
                elif cmd == "POS":
                    coords = [float(c) for c in val.split(',')]
                    self.robot_current_jpos = coords
                    self.lbl_j1_pos.setText(f"J1: {coords[0]:.2f}°"); self.lbl_j2_pos.setText(f"J2: {coords[1]:.2f}mm")
                    self.lbl_j3_pos.setText(f"J3: {coords[2]:.2f}°"); self.lbl_j4_pos.setText(f"J4: {coords[3]:.2f}°")
                elif cmd == "GRIPPER":
                    self.update_gripper_ui(int(val))

        except Exception as e:
            self.show_status_message(f"Erro de processamento serial: {e}", "error")

    def translate_arduino_message(self, data):
        """Traduz as mensagens do Arduino para um formato mais amigável."""
        error_translations = {
            "ERR:NOT_READY": "Robô não está pronto. Calibre e ligue os motores.",
            "ERR:LIMIT_J1": "Movimento fora dos limites da Junta 1 (Base).", "ERR:LIMIT_J2": "Movimento fora dos limites da Junta 2 (Altura).",
            "ERR:LIMIT_J3": "Movimento fora dos limites da Junta 3 (Cotovelo).", "ERR:LIMIT_J4": "Movimento fora dos limites da Junta 4 (Garra).",
            "ERR:UNREACHABLE_OR_LIMITS": "Posição inalcançável ou fora dos limites.", "MSG:PARAGEM DE EMERGENCIA.": "Paragem de emergência acionada!",
        }
        if data in error_translations: return {"text": f"ERRO: {error_translations[data]}", "level": "error"}
        translations = {
            "MSG:Conexão estabelecida.": ("Conectado ao robô com sucesso.", "success"), "MSG:Robo SCARA pronto.": ("Robô pronto.", "success"),
            "MSG:Movimento concluido.": ("Movimento concluído.", "success"), "MSG:Calibracao concluida.": ("Calibração concluída.", "success"),
            "MSG:Iniciando auto-calibracao...": ("A iniciar a calibração automática...", "info"), "MSG:Motores LIGADOS": ("Motores ligados.", "info"),
            "MSG:Motores DESLIGADOS": ("Motores desligados.", "warning"), "MSG:Garra aberta.": ("Garra aberta.", "info"), "MSG:Garra fechada.": ("Garra fechada.", "info"),
        }
        if data in translations: text, level = translations[data]; return {"text": text, "level": level}
        if data.startswith("MSG:"): return {"text": data[4:].strip(), "level": "info"}
        if data.startswith("ERR:"): return {"text": f"ERRO: {data[4:].strip()}", "level": "error"}
        return {"text": data, "level": "normal"}

    def on_jog_button_pressed(self):
        self.current_jog_command = self.sender().property("jog_command")
        self.execute_jog_step()
        self.jog_timer.start()

    def on_jog_button_released(self):
        self.current_jog_command = None; self.jog_timer.stop()

    def execute_jog_step(self):
        if not self.current_jog_command: return
        axis, direction = self.current_jog_command.split(','); direction = int(direction)
        x, y, z, u = self.robot_current_wpos
        if axis == 'X': x += JOG_STEP_XY * direction
        elif axis == 'Y': y += JOG_STEP_XY * direction
        elif axis == 'Z': z += JOG_STEP_Z * direction
        elif axis == 'U': u += JOG_STEP_U * direction
        self.send_command(f"GOTO_IK:{x},{y},{z},{u}\n")

    def change_speed(self, delta):
        try: current_pct = int(self.lbl_speed.text().split(':')[1].strip().replace('%',''))
        except: current_pct = 100
        new_pct = max(50, min(current_pct + (delta * 100), 150))
        self.lbl_speed.setText(f"Vel: {int(new_pct)}%")
        self.send_command(f"SET_SPEED_MULT:{new_pct / 100.0:.2f}\n")

    def on_slider_released(self):
        pos = {name: s_dict["slider"].value() / 10.0 for name, s_dict in self.sliders.items()}
        self.show_status_message(f"Movimentando juntas para J1:{pos['J1']:.1f}, J2:{pos['J2']:.1f}, J3:{pos['J3']:.1f}, J4:{pos['J4']:.1f}...", "info")
        self.send_command(f"GOTO_JOINTS:{pos['J1']},{pos['J2']},{pos['J3']},{pos['J4']}\n")

    def toggle_motors(self):
        self.send_command(f"MOTORS:{0 if self.motors_on else 1}\n")
        
    def update_motors_button_ui(self, is_on):
        self.motors_on = is_on
        self.btn_toggle_motors.setToolTip("Motores Ligados" if is_on else "Motores Desligados")
        if is_on:
            self.btn_toggle_motors.setStyleSheet("background-color: #A3BE8C; font-size: 16pt; font-weight: bold;")
        else:
            self.btn_toggle_motors.setStyleSheet("background-color: #BF616A; font-size: 16pt; font-weight: bold;")

    def show_status_message(self, text, level="normal"):
        styles = {
            "normal": "color: black;",
            "info": "color: blue;",
            "success": "color: green;",
            "warning": "color: orange;",
            "error": "color: red;"
        }
        self.status_label.setStyleSheet(f"font-weight: bold; {styles.get(level, styles['normal'])}")
        self.status_label.setText(text)
        
        if level in ["warning", "error", "success"]:
            QTimer.singleShot(4000, lambda: self.show_status_message("Pronto.", "normal"))
    
    def closeEvent(self, event):
        self.worker.stop(); self.thread.quit(); self.thread.wait(); event.accept()
    
    @Slot(int)
    def on_tab_changed(self, index):
        if self.tabs.tabText(index) == "Controlo FK (Juntas Individuais)":
            self.update_fk_sliders_from_robot_state()

    def update_fk_sliders_from_robot_state(self):
        if not self.robot_current_jpos: return
        joint_keys = ["J1", "J2", "J3", "J4"]
        for i, key in enumerate(joint_keys):
            slider = self.sliders[key]["slider"]
            slider.setValue(int(self.robot_current_jpos[i] * 10))

    def on_gripper_open_clicked(self):
        self.send_command("GRIPPER_OPEN\n")
        self.update_gripper_ui(self.GRIPPER_ANGLE_OPEN)

    def on_gripper_close_clicked(self):
        self.send_command("GRIPPER_CLOSE\n")
        self.update_gripper_ui(self.GRIPPER_ANGLE_CLOSE)

    def on_gripper_slider_moved(self, value):
        mm_value = value / 10.0
        self.lbl_gripper_pos.setText(f"{mm_value:.1f} mm")

    def on_gripper_slider_released(self):
        mm_value = self.gripper_slider.value() / 10.0
        angle_to_set = self.map_mm_to_angle(mm_value)
        self.send_command(f"GRIPPER_SET:{angle_to_set}\n")
        self.show_status_message(f"Garra ajustada para {mm_value:.1f} mm", "info")

    def export_sequences(self):
        filePath, _ = QFileDialog.getSaveFileName(self, "Guardar Sequências", "", "Ficheiro de Texto (*.txt);;Todos os Ficheiros (*)")
        if filePath:
            try:
                with open(filePath, 'w', encoding='utf-8') as f:
                    json.dump(self.sequences, f, indent=4)
                self.show_status_message(f"Sequências guardadas em {Path(filePath).name}", "success")
            except Exception as e:
                self.show_status_message(f"Erro ao guardar: {e}", "error")

    def import_sequences(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "Abrir Sequências", "", "Ficheiro de Texto (*.txt);;Todos os Ficheiros (*)")
        if filePath:
            try:
                with open(filePath, 'r', encoding='utf-8') as f:
                    loaded_data = json.load(f)
                
                if not isinstance(loaded_data, dict):
                    raise ValueError("O ficheiro não contém um dicionário de sequências válido.")
                
                self.sequences = {int(k): v for k, v in loaded_data.items()}
                
                for i in range(1, 4):
                    if i not in self.sequences:
                        self.sequences[i] = {"name": f"Sequência {i}", "positions": [None] * MAX_SEQUENCE_POSITIONS}
                    else:
                        for j, pos in enumerate(self.sequences[i]["positions"]):
                            if pos and 'gripper' not in pos:
                                self.sequences[i]["positions"][j]['gripper'] = self.GRIPPER_MM_OPEN
                        if len(self.sequences[i]["positions"]) < MAX_SEQUENCE_POSITIONS:
                            self.sequences[i]["positions"].extend([None] * (MAX_SEQUENCE_POSITIONS - len(self.sequences[i]["positions"])))

                for i in range(1, 4):
                    self.update_seq_ui(i)
                    
                self.show_status_message(f"Sequências carregadas de {Path(filePath).name}", "success")

            except Exception as e:
                self.show_status_message(f"Erro ao abrir: {e}", "error")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HMI_Completa()
    window.show()
    sys.exit(app.exec())
