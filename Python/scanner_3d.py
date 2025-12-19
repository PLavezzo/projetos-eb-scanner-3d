#!/usr/bin/env python3
"""
Scanner 3D - Interface Principal
Sistema de captura 3D usando sensor HC-SR04 e Arduino
"""

import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QComboBox, 
                              QGroupBox, QMessageBox, QGridLayout, QLineEdit)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import serial
import serial.tools.list_ports
from vispy import scene
from vispy.scene import visuals
from vispy.visuals.transforms import MatrixTransform
from stl import mesh as stl_mesh


class Scanner3D(QMainWindow):
    """Janela principal do Scanner 3D"""
    
    def __init__(self):
        super().__init__()
        
        # Parâmetros do scanner
        self.POINTS_PER_LAYER = 7  # Pontos por volta completa
        self.LAYER_HEIGHT = 8.0    # mm - altura entre camadas
        self.ANGLE_STEP = 360.0 / self.POINTS_PER_LAYER  # ~51.43 graus
        self.ARM_RADIUS = 200.0    # mm - distância do centro até o sensor (RAIO DO BRAÇO)
        
        # Estado da coleta
        self.current_point = 0      # Ponto atual (0-6)
        self.current_layer = 0      # Camada atual
        self.current_distance = 0.0 # Última distância lida
        self.is_collecting = False  # Flag de coleta ativa
        
        # Dados coletados: lista de (x, y, z) em mm
        self.scan_data = []
        
        # Serial
        self.serial_port = None
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial)
        
        # Inicializa interface
        self.init_ui()
        
    def init_ui(self):
        """Inicializa interface do usuário"""
        self.setWindowTitle("Scanner 3D - HC-SR04")
        self.setGeometry(100, 100, 1400, 800)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal (horizontal)
        main_layout = QHBoxLayout(central_widget)
        
        # === PAINEL ESQUERDO - Controles ===
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)
        
        # === PAINEL DIREITO - Visualização 3D ===
        viz_panel = self.create_visualization_panel()
        main_layout.addWidget(viz_panel, stretch=2)
        
    def create_control_panel(self):
        """Cria painel de controle"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(15)
        
        # Título
        title = QLabel("Scanner 3D")
        title.setFont(QFont("Arial", 20, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        # === Conexão Serial ===
        serial_group = QGroupBox("Conexão")
        serial_layout = QVBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(QLabel("Porta Serial:"))
        serial_layout.addWidget(self.port_combo)
        
        port_btn_layout = QHBoxLayout()
        self.refresh_btn = QPushButton("🔄 Atualizar")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 8px; }")
        
        port_btn_layout.addWidget(self.refresh_btn)
        port_btn_layout.addWidget(self.connect_btn)
        serial_layout.addLayout(port_btn_layout)
        
        self.connection_status = QLabel("● Desconectado")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        serial_layout.addWidget(self.connection_status)
        
        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)
        
        # === Configurações do Scanner ===
        config_group = QGroupBox("Configurações")
        config_layout = QGridLayout()
        
        config_layout.addWidget(QLabel("Raio do Braço (mm):"), 0, 0)
        self.arm_radius_input = QLineEdit(str(self.ARM_RADIUS))
        self.arm_radius_input.setMaximumWidth(80)
        self.arm_radius_input.editingFinished.connect(self.update_arm_radius)
        config_layout.addWidget(self.arm_radius_input, 0, 1)
        
        arm_info = QLabel("↳ Distância centro → sensor")
        arm_info.setStyleSheet("color: #666; font-size: 10px;")
        config_layout.addWidget(arm_info, 1, 0, 1, 2)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)
        
        # === Status da Coleta ===
        status_group = QGroupBox("Status da Coleta")
        status_layout = QGridLayout()
        
        status_layout.addWidget(QLabel("Camada:"), 0, 0)
        self.layer_label = QLabel("0")
        self.layer_label.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        self.layer_label.setStyleSheet("color: #2196F3;")
        status_layout.addWidget(self.layer_label, 0, 1)
        
        status_layout.addWidget(QLabel("Ponto:"), 1, 0)
        self.point_label = QLabel("1 / 7")
        self.point_label.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        self.point_label.setStyleSheet("color: #2196F3;")
        status_layout.addWidget(self.point_label, 1, 1)
        
        status_layout.addWidget(QLabel("Altura:"), 2, 0)
        self.height_label = QLabel("0.0 mm")
        self.height_label.setFont(QFont("Arial", 14))
        status_layout.addWidget(self.height_label, 2, 1)
        
        status_layout.addWidget(QLabel("Distância:"), 3, 0)
        self.distance_label = QLabel("--- cm")
        self.distance_label.setFont(QFont("Arial", 14))
        status_layout.addWidget(self.distance_label, 3, 1)
        
        status_layout.addWidget(QLabel("Total Pontos:"), 4, 0)
        self.total_points_label = QLabel("0")
        self.total_points_label.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        self.total_points_label.setStyleSheet("color: #FF9800;")
        status_layout.addWidget(self.total_points_label, 4, 1)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # === Controles de Coleta ===
        collect_group = QGroupBox("Controles")
        collect_layout = QVBoxLayout()
        
        self.collect_btn = QPushButton("📍 Coletar Ponto")
        self.collect_btn.setEnabled(False)
        self.collect_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 12px; font-size: 14px; }")
        self.collect_btn.clicked.connect(self.collect_point)
        collect_layout.addWidget(self.collect_btn)
        
        # Botão Anterior
        self.prev_btn = QPushButton("⬅️ Ponto Anterior")
        self.prev_btn.setEnabled(False)
        self.prev_btn.setStyleSheet("QPushButton { padding: 10px; font-size: 13px; background-color: #9E9E9E; color: white; }")
        self.prev_btn.clicked.connect(self.previous_point)
        collect_layout.addWidget(self.prev_btn)
        
        self.finish_btn = QPushButton("✅ Finalizar Scan")
        self.finish_btn.setEnabled(False)
        self.finish_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; padding: 10px; font-size: 13px; }")
        self.finish_btn.clicked.connect(self.finish_scan)
        collect_layout.addWidget(self.finish_btn)
        
        collect_group.setLayout(collect_layout)
        layout.addWidget(collect_group)
        
        # === Botões de Ação ===
        action_layout = QVBoxLayout()
        
        self.clear_btn = QPushButton("🗑️ Limpar Dados")
        self.clear_btn.clicked.connect(self.clear_data)
        action_layout.addWidget(self.clear_btn)
        
        self.export_btn = QPushButton("💾 Exportar Dados")
        self.export_btn.clicked.connect(self.export_data)
        action_layout.addWidget(self.export_btn)
        
        layout.addLayout(action_layout)
        
        # Espaçador
        layout.addStretch()
        
        return panel
    
    def create_visualization_panel(self):
        """Cria painel de visualização 3D"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Canvas Vispy com fundo gradiente escuro
        self.canvas = scene.SceneCanvas(
            keys='interactive', 
            show=True, 
            bgcolor='#1a1a2e',
            size=(800, 600)
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.TurntableCamera(
            elevation=25, 
            azimuth=45, 
            distance=400,
            fov=50
        )
        
        # Grid 3D bonito
        self.grid = scene.visuals.GridLines(
            parent=self.view.scene,
            color=(0.3, 0.3, 0.4, 0.6)
        )
        
        # Eixos XYZ maiores e coloridos
        self.axis = scene.visuals.XYZAxis(
            parent=self.view.scene,
            width=3
        )
        
        # Nuvem de pontos - inicializada vazia
        self.scatter = scene.visuals.Markers(parent=self.view.scene)
        self.scatter.set_data(pos=np.zeros((0, 3)))
        
        # Mesh que conecta os pontos (superfície) - inicializada vazia
        self.mesh = scene.visuals.Mesh(
            parent=self.view.scene,
            shading='smooth'
        )
        self.mesh.visible = False
        
        # Wireframe - inicializado vazio
        self.wireframe = scene.visuals.Line(
            parent=self.view.scene,
            color=(0.3, 0.8, 1.0, 0.4),
            width=1.5
        )
        self.wireframe.visible = False
        
        layout.addWidget(self.canvas.native)
        
        # Controles da visualização
        viz_controls = QHBoxLayout()
        
        self.show_points_btn = QPushButton("● Pontos")
        self.show_points_btn.setCheckable(True)
        self.show_points_btn.setChecked(True)
        self.show_points_btn.clicked.connect(self.toggle_points)
        
        self.show_mesh_btn = QPushButton("▲ Superfície")
        self.show_mesh_btn.setCheckable(True)
        self.show_mesh_btn.setChecked(True)
        self.show_mesh_btn.clicked.connect(self.toggle_mesh)
        
        self.show_wire_btn = QPushButton("⚡ Wireframe")
        self.show_wire_btn.setCheckable(True)
        self.show_wire_btn.setChecked(True)
        self.show_wire_btn.clicked.connect(self.toggle_wireframe)
        
        viz_controls.addWidget(self.show_points_btn)
        viz_controls.addWidget(self.show_mesh_btn)
        viz_controls.addWidget(self.show_wire_btn)
        layout.addLayout(viz_controls)
        
        # Info sobre visualização
        info = QLabel("🖱️ Arraste: Rotacionar | Scroll: Zoom | Shift+Arraste: Pan")
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info.setStyleSheet("color: #888; font-size: 11px; padding: 5px;")
        layout.addWidget(info)
        
        return panel
    
    def toggle_points(self):
        """Toggle visibilidade dos pontos"""
        self.scatter.visible = self.show_points_btn.isChecked()
        self.canvas.update()
    
    def toggle_mesh(self):
        """Toggle visibilidade da superfície"""
        self.mesh.visible = self.show_mesh_btn.isChecked()
        self.canvas.update()
    
    def toggle_wireframe(self):
        """Toggle visibilidade do wireframe"""
        self.wireframe.visible = self.show_wire_btn.isChecked()
        self.canvas.update()
    
    def update_arm_radius(self):
        """Atualiza raio do braço quando usuário muda o valor"""
        try:
            new_radius = float(self.arm_radius_input.text())
            if new_radius > 0:
                self.ARM_RADIUS = new_radius
                print(f"Raio do braço atualizado para: {self.ARM_RADIUS} mm")
            else:
                QMessageBox.warning(self, "Valor Inválido", "O raio deve ser maior que zero!")
                self.arm_radius_input.setText(str(self.ARM_RADIUS))
        except ValueError:
            QMessageBox.warning(self, "Valor Inválido", "Digite um número válido!")
            self.arm_radius_input.setText(str(self.ARM_RADIUS))
    
    def refresh_ports(self):
        """Atualiza lista de portas seriais disponíveis"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # No Mac, priorizar portas USB
            if 'usb' in port.device.lower() or 'cu.' in port.device.lower():
                self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
    
    def toggle_connection(self):
        """Conecta ou desconecta da porta serial"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """Conecta à porta serial"""
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "Erro", "Nenhuma porta serial encontrada!")
            return
        
        port_name = self.port_combo.currentData()
        
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=115200,
                timeout=0.1
            )
            
            # Aguarda inicialização do Arduino
            import time
            time.sleep(2)
            
            # Limpa buffer
            self.serial_port.reset_input_buffer()
            
            # Inicia timer de leitura (100ms - 10Hz)
            self.serial_timer.start(100)
            
            # Atualiza UI
            self.connection_status.setText("● Conectado")
            self.connection_status.setStyleSheet("color: green; font-weight: bold;")
            self.connect_btn.setText("Desconectar")
            self.connect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 8px; }")
            
            self.collect_btn.setEnabled(True)
            self.finish_btn.setEnabled(True)
            
            QMessageBox.information(self, "Sucesso", "Conectado ao Arduino!")
            
        except Exception as e:
            QMessageBox.critical(self, "Erro de Conexão", f"Não foi possível conectar:\n{str(e)}")
    
    def disconnect_serial(self):
        """Desconecta da porta serial"""
        self.serial_timer.stop()
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.connection_status.setText("● Desconectado")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Conectar")
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 8px; }")
        
        self.collect_btn.setEnabled(False)
        self.prev_btn.setEnabled(False)
        self.finish_btn.setEnabled(False)
    
    def read_serial(self):
        """Lê dados da porta serial"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if line.startswith("DIST:"):
                    distance_str = line.split(":")[1]
                    
                    if distance_str != "ERROR":
                        try:
                            self.current_distance = float(distance_str)
                            self.distance_label.setText(f"{self.current_distance:.2f} cm")
                            self.distance_label.setStyleSheet("color: green;")
                        except ValueError:
                            pass
                    else:
                        self.distance_label.setText("ERRO")
                        self.distance_label.setStyleSheet("color: red;")
                        
        except Exception as e:
            print(f"Erro lendo serial: {e}")
    
    def collect_point(self):
        """Coleta o ponto atual e avança automaticamente"""
        if self.current_distance <= 0:
            QMessageBox.warning(self, "Aviso", "Aguarde leitura válida do sensor!")
            return
        
        # Calcula coordenadas cilíndricas
        # Usa o current_point ANTES de incrementar
        angle_deg = self.current_point * self.ANGLE_STEP
        angle_rad = np.radians(angle_deg)
        height_mm = self.current_layer * self.LAYER_HEIGHT
        
        # Distância do sensor até o objeto (em mm)
        sensor_distance_mm = self.current_distance * 10.0  # cm para mm
        
        # RAIO EFETIVO = Raio do braço - distância medida
        # Quanto MENOR a distância do sensor, MAIOR o raio (mais próximo da parede externa)
        # Quanto MAIOR a distância do sensor, MENOR o raio (mais próximo do centro)
        effective_radius_mm = self.ARM_RADIUS - sensor_distance_mm
        
        # Converte para coordenadas cartesianas (x, y, z)
        x = effective_radius_mm * np.cos(angle_rad)
        y = effective_radius_mm * np.sin(angle_rad)
        z = height_mm
        
        # Log detalhado ANTES de adicionar
        print(f"\n=== COLETANDO PONTO ===")
        print(f"Camada: {self.current_layer} | Ponto: {self.current_point + 1}/{self.POINTS_PER_LAYER}")
        print(f"Ângulo: {angle_deg:.2f}° ({angle_rad:.4f} rad)")
        print(f"Raio do braço: {self.ARM_RADIUS:.1f} mm")
        print(f"Distância sensor: {self.current_distance:.2f} cm ({sensor_distance_mm:.1f} mm)")
        print(f"Raio efetivo: {effective_radius_mm:.1f} mm")
        print(f"Altura: {height_mm:.1f} mm")
        print(f"Coordenadas cartesianas: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        
        # Adiciona ponto
        self.scan_data.append([x, y, z])
        print(f"Total de pontos: {len(self.scan_data)}")
        
        # Atualiza UI
        self.total_points_label.setText(str(len(self.scan_data)))
        
        # AVANÇA AUTOMATICAMENTE para o próximo ponto
        old_point = self.current_point
        old_layer = self.current_layer
        
        self.current_point += 1
        
        # Verifica se completou a camada
        if self.current_point >= self.POINTS_PER_LAYER:
            self.current_point = 0
            self.current_layer += 1
            
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Icon.Information)
            msg.setWindowTitle("Camada Completa")
            msg.setText(f"Camada {self.current_layer - 1} concluída com {self.POINTS_PER_LAYER} pontos!")
            msg.setInformativeText(f"Estenda o braço em {self.LAYER_HEIGHT}mm e clique OK para continuar na próxima camada (Camada {self.current_layer}).")
            msg.exec()
        
        print(f"Avançou de Ponto {old_point + 1} -> {self.current_point + 1} (Camada {self.current_layer})")
        
        # Atualiza visualização DEPOIS de adicionar o ponto
        self.update_visualization()
        
        # Atualiza display
        self.update_status_display()
        
        # Habilita botão anterior
        self.prev_btn.setEnabled(True)
    
    def previous_point(self):
        """Volta para o ponto anterior e remove última coleta"""
        if len(self.scan_data) == 0:
            QMessageBox.warning(self, "Aviso", "Nenhum ponto coletado ainda!")
            return
        
        # Remove último ponto
        self.scan_data.pop()
        
        # Atualiza contadores
        self.current_point -= 1
        
        # Se voltou de camada
        if self.current_point < 0:
            if self.current_layer > 0:
                self.current_layer -= 1
                self.current_point = self.POINTS_PER_LAYER - 1
                QMessageBox.information(
                    self, 
                    "Camada Anterior", 
                    f"Voltou para camada {self.current_layer}, ponto {self.current_point + 1}"
                )
            else:
                self.current_point = 0
        
        # Atualiza visualização e UI
        self.update_visualization()
        self.update_status_display()
        self.total_points_label.setText(str(len(self.scan_data)))
        
        # Desabilita se não houver mais pontos
        if len(self.scan_data) == 0:
            self.prev_btn.setEnabled(False)
        
        print(f"Ponto removido. Agora em: Camada {self.current_layer}, Ponto {self.current_point + 1}")
    
    def next_point(self):
        """REMOVIDO - agora o avanço é automático após coletar"""
        pass
    
    def update_status_display(self):
        """Atualiza labels de status"""
        self.layer_label.setText(str(self.current_layer))
        self.point_label.setText(f"{self.current_point + 1} / {self.POINTS_PER_LAYER}")
        self.height_label.setText(f"{self.current_layer * self.LAYER_HEIGHT:.1f} mm")
    
    def update_visualization(self):
        """Atualiza visualização 3D com pontos, mesh e wireframe"""
        print(f"\n>>> Atualizando visualização com {len(self.scan_data)} pontos...")
        
        if len(self.scan_data) == 0:
            # Limpa visualização de forma segura
            self.scatter.set_data(pos=np.zeros((0, 3)))
            
            # Limpa mesh e wireframe verificando se existem
            try:
                self.mesh.visible = False
                self.wireframe.visible = False
            except:
                pass
            
            self.canvas.update()
            return
        
        # Converte para array numpy
        points = np.array(self.scan_data)
        print(f"Array de pontos shape: {points.shape}")
        print(f"Primeiros 3 pontos:\n{points[:min(3, len(points))]}")
        
        # ===== PONTOS COM CORES BASEADAS NA ALTURA =====
        if len(points) > 1:
            z_min, z_max = points[:, 2].min(), points[:, 2].max()
            if z_max > z_min:
                norm_z = (points[:, 2] - z_min) / (z_max - z_min)
            else:
                norm_z = np.ones(len(points)) * 0.5
        else:
            norm_z = np.array([0.5])
        
        # Gradiente vibrante: Azul (baixo) -> Ciano -> Verde -> Amarelo -> Vermelho (alto)
        color_map = np.zeros((len(points), 4))
        for i, z in enumerate(norm_z):
            if z < 0.25:
                # Azul -> Ciano
                t = z / 0.25
                color_map[i] = [0.0, t, 1.0, 1.0]
            elif z < 0.5:
                # Ciano -> Verde
                t = (z - 0.25) / 0.25
                color_map[i] = [0.0, 1.0, 1.0 - t, 1.0]
            elif z < 0.75:
                # Verde -> Amarelo
                t = (z - 0.5) / 0.25
                color_map[i] = [t, 1.0, 0.0, 1.0]
            else:
                # Amarelo -> Vermelho
                t = (z - 0.75) / 0.25
                color_map[i] = [1.0, 1.0 - t, 0.0, 1.0]
        
        # Atualiza pontos - SEMPRE MOSTRA TODOS
        try:
            self.scatter.set_data(
                pos=points, 
                face_color=color_map,
                edge_color='white',
                edge_width=1.5,
                size=12
            )
            self.scatter.visible = True
            print(f"✓ Scatter atualizado com {len(points)} pontos")
        except Exception as e:
            print(f"✗ Erro atualizando scatter: {e}")
        
        # ===== WIREFRAME (conecta pontos na mesma camada) =====
        num_layers = self.current_layer + 1
        points_per_layer = self.POINTS_PER_LAYER
        
        wire_lines = []
        
        # Conecta pontos em cada camada (forma círculos)
        for layer in range(num_layers):
            layer_start = layer * points_per_layer
            layer_end = min(layer_start + points_per_layer, len(points))
            layer_count = layer_end - layer_start
            
            # Conecta pontos consecutivos na camada
            for i in range(layer_count):
                p1_idx = layer_start + i
                # Próximo ponto (fecha o círculo se camada completa)
                if i < layer_count - 1:
                    p2_idx = layer_start + i + 1
                    wire_lines.extend([points[p1_idx], points[p2_idx]])
                elif layer_count == points_per_layer:
                    # Fecha o círculo
                    p2_idx = layer_start
                    wire_lines.extend([points[p1_idx], points[p2_idx]])
        
        # Conecta camadas verticalmente
        if num_layers > 1:
            for i in range(min(points_per_layer, len(points))):
                for layer in range(num_layers - 1):
                    p1_idx = layer * points_per_layer + i
                    p2_idx = (layer + 1) * points_per_layer + i
                    if p2_idx < len(points):
                        wire_lines.extend([points[p1_idx], points[p2_idx]])
        
        if len(wire_lines) > 0:
            try:
                wire_array = np.array(wire_lines)
                self.wireframe.set_data(pos=wire_array, connect='segments', color=(0.3, 0.8, 1.0, 0.8), width=2)
                self.wireframe.visible = True
                print(f"✓ Wireframe criado com {len(wire_lines)//2} linhas")
            except Exception as e:
                print(f"✗ Erro criando wireframe: {e}")
                self.wireframe.visible = False
        else:
            self.wireframe.visible = False
        
        # ===== MESH SURFACE (se houver pelo menos 2 camadas completas) =====
        if len(points) >= points_per_layer * 2:
            try:
                max_complete_layers = len(points) // points_per_layer
                grid_points = points[:max_complete_layers * points_per_layer]
                
                # Cria faces conectando pontos em camadas adjacentes
                faces = []
                for layer in range(max_complete_layers - 1):
                    for i in range(points_per_layer):
                        i_next = (i + 1) % points_per_layer
                        
                        p1 = layer * points_per_layer + i
                        p2 = layer * points_per_layer + i_next
                        p3 = (layer + 1) * points_per_layer + i
                        p4 = (layer + 1) * points_per_layer + i_next
                        
                        # Dois triângulos por quad
                        faces.append([p1, p2, p3])
                        faces.append([p2, p4, p3])
                
                if len(faces) > 0:
                    faces = np.array(faces, dtype=np.uint32)
                    
                    # Cores dos vértices
                    vertex_colors = np.zeros((len(grid_points), 4))
                    if len(grid_points) > 0:
                        z_vals = grid_points[:, 2]
                        z_min_m, z_max_m = z_vals.min(), z_vals.max()
                        if z_max_m > z_min_m:
                            norm_z_m = (z_vals - z_min_m) / (z_max_m - z_min_m)
                        else:
                            norm_z_m = np.ones(len(z_vals)) * 0.5
                        
                        for i, z in enumerate(norm_z_m):
                            if z < 0.25:
                                t = z / 0.25
                                vertex_colors[i] = [0.0, t, 1.0, 0.6]
                            elif z < 0.5:
                                t = (z - 0.25) / 0.25
                                vertex_colors[i] = [0.0, 1.0, 1.0 - t, 0.6]
                            elif z < 0.75:
                                t = (z - 0.5) / 0.25
                                vertex_colors[i] = [t, 1.0, 0.0, 0.6]
                            else:
                                t = (z - 0.75) / 0.25
                                vertex_colors[i] = [1.0, 1.0 - t, 0.0, 0.6]
                    
                    self.mesh.set_data(
                        vertices=grid_points,
                        faces=faces,
                        vertex_colors=vertex_colors
                    )
                    self.mesh.visible = True
                    print(f"✓ Mesh criado com {len(grid_points)} vértices e {len(faces)} faces")
            except Exception as e:
                print(f"✗ Erro criando mesh: {e}")
                self.mesh.visible = False
        else:
            self.mesh.visible = False
        
        # Força atualização do canvas
        self.canvas.update()
        print(">>> Visualização atualizada!\n")
        self.scatter.set_data(points, face_color=color_map, size=8, edge_width=0)
    
    def clear_data(self):
        """Limpa todos os dados coletados"""
        reply = QMessageBox.question(
            self, 
            "Confirmar Limpeza", 
            "Deseja realmente limpar todos os dados coletados?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.scan_data = []
            self.current_point = 0
            self.current_layer = 0
            self.update_status_display()
            self.total_points_label.setText("0")
            
            # Limpa visualização de forma segura
            self.scatter.set_data(pos=np.zeros((0, 3)))
            self.mesh.visible = False
            self.wireframe.visible = False
            self.canvas.update()
            
            self.prev_btn.setEnabled(False)
    
    def finish_scan(self):
        """Finaliza o scan"""
        if len(self.scan_data) == 0:
            QMessageBox.warning(self, "Aviso", "Nenhum dado coletado ainda!")
            return
        
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Icon.Question)
        msg.setWindowTitle("Finalizar Scan")
        msg.setText(f"Scan finalizado com {len(self.scan_data)} pontos!")
        msg.setInformativeText("Deseja exportar os dados agora?")
        msg.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        
        reply = msg.exec()
        
        if reply == QMessageBox.StandardButton.Yes:
            self.export_data()
    
    def export_data(self):
        """Exporta dados coletados em CSV e STL"""
        if len(self.scan_data) == 0:
            QMessageBox.warning(self, "Aviso", "Nenhum dado para exportar!")
            return
        
        from datetime import datetime
        import os
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"scan_3d_{timestamp}"
        
        # === EXPORTAR CSV ===
        csv_filename = f"{base_filename}.csv"
        csv_filepath = os.path.join(os.path.dirname(__file__), csv_filename)
        
        try:
            with open(csv_filepath, 'w') as f:
                f.write("X(mm),Y(mm),Z(mm)\n")
                for point in self.scan_data:
                    f.write(f"{point[0]:.3f},{point[1]:.3f},{point[2]:.3f}\n")
            
            csv_saved = True
        except Exception as e:
            csv_saved = False
            csv_error = str(e)
        
        # === EXPORTAR STL ===
        stl_saved = False
        stl_error = ""
        
        # Só cria STL se tiver pelo menos 2 camadas completas
        points_per_layer = self.POINTS_PER_LAYER
        if len(self.scan_data) >= points_per_layer * 2:
            try:
                points = np.array(self.scan_data)
                max_complete_layers = len(points) // points_per_layer
                grid_points = points[:max_complete_layers * points_per_layer]
                
                # Cria faces (mesma lógica do mesh)
                faces = []
                for layer in range(max_complete_layers - 1):
                    for i in range(points_per_layer):
                        i_next = (i + 1) % points_per_layer
                        
                        p1 = layer * points_per_layer + i
                        p2 = layer * points_per_layer + i_next
                        p3 = (layer + 1) * points_per_layer + i
                        p4 = (layer + 1) * points_per_layer + i_next
                        
                        # Dois triângulos por quad
                        faces.append([p1, p2, p3])
                        faces.append([p2, p4, p3])
                
                if len(faces) > 0:
                    faces_array = np.array(faces)
                    
                    # Cria mesh STL
                    scan_mesh = stl_mesh.Mesh(np.zeros(faces_array.shape[0], dtype=stl_mesh.Mesh.dtype))
                    
                    for i, face in enumerate(faces_array):
                        for j in range(3):
                            scan_mesh.vectors[i][j] = grid_points[face[j]]
                    
                    # Salva arquivo STL
                    stl_filename = f"{base_filename}.stl"
                    stl_filepath = os.path.join(os.path.dirname(__file__), stl_filename)
                    scan_mesh.save(stl_filepath)
                    
                    stl_saved = True
                else:
                    stl_error = "Não há faces suficientes para criar mesh"
                    
            except Exception as e:
                stl_error = str(e)
        else:
            stl_error = f"Necessário pelo menos {points_per_layer * 2} pontos (2 camadas completas)"
        
        # === MENSAGEM FINAL ===
        message_parts = []
        
        if csv_saved:
            message_parts.append(f"✓ CSV: {csv_filename}")
        else:
            message_parts.append(f"✗ CSV falhou: {csv_error}")
        
        if stl_saved:
            message_parts.append(f"✓ STL: {stl_filename}")
        else:
            message_parts.append(f"✗ STL: {stl_error}")
        
        message_parts.append(f"\nTotal: {len(self.scan_data)} pontos")
        
        if csv_saved or stl_saved:
            QMessageBox.information(
                self, 
                "Exportação Concluída", 
                "\n".join(message_parts)
            )
        else:
            QMessageBox.critical(
                self, 
                "Erro na Exportação", 
                "\n".join(message_parts)
            )
    
    def closeEvent(self, event):
        """Evento de fechamento da janela"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Estilo da aplicação
    app.setStyle('Fusion')
    
    window = Scanner3D()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
