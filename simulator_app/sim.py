import sys
import time
import pathlib
import math
from stl import mesh
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QSlider, QCheckBox, QGroupBox, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from collections import deque

# --- GENERATORY MODELI 3D ---
def create_box_mesh(length, width, thickness):
    # Wierzchołki prostopadłościanu
    v = np.array([
        [0, -width/2, -thickness/2],      # 0: Lewy Przód Dół
        [length, -width/2, -thickness/2], # 1: Prawy Przód Dół
        [length, width/2, -thickness/2],  # 2: Prawy Tył Dół
        [0, width/2, -thickness/2],       # 3: Lewy Tył Dół
        [0, -width/2, thickness/2],       # 4: Lewy Przód Góra
        [length, -width/2, thickness/2],  # 5: Prawy Przód Góra
        [length, width/2, thickness/2],   # 6: Prawy Tył Góra
        [0, width/2, thickness/2]         # 7: Lewy Tył Góra
    ])
    
    # POPRAWIONA KOLEJNOŚĆ (Winding order) - wektory normalne skierowane na zewnątrz
    f = np.array([
        [4, 5, 6], [4, 6, 7], # Góra
        [0, 2, 1], [0, 3, 2], # Dół
        [0, 1, 5], [0, 5, 4], # Przód
        [3, 6, 2], [3, 7, 6], # Tył
        [0, 4, 7], [0, 7, 3], # Lewy bok
        [1, 2, 6], [1, 6, 5]  # Prawy bok
    ])
    
    return gl.MeshData(vertexes=v, faces=f)

def create_cylinder_mesh(radius, thickness):
    # Tworzy walec dla złączy/przegubów
    md = gl.MeshData.cylinder(rows=1, cols=20, radius=[radius, radius], length=thickness)
    # Przesunięcie walca, by środek był w 0,0,0
    v = md.vertexes()
    v[:, 2] -= thickness/2
    md.setVertexes(v)
    return md

def load_stl_mesh(filepath, color=(0.5, 0.5, 0.5, 1.0)):
    # 1. Wczytanie pliku STL
    stl_data = mesh.Mesh.from_file(filepath)

    # 2. STL przechowuje dane w formacie wektorów (N, 3, 3) gdzie N to trójkąty.
    # Musimy to "spłaszczyć" do listy punktów w przestrzeni (Wierzchołki).
    points = stl_data.vectors.reshape(-1, 3)

    # 3. Tworzymy listę "ścian" (Faces). Skoro spłaszczyliśmy punkty, 
    # każdy trójkąt to po prostu kolejne 3 indeksy: (0,1,2), (3,4,5)...
    faces = np.arange(len(points)).reshape(-1, 3)

    # 4. Pakujemy to w format PyQtGraph
    mesh_data = gl.MeshData(vertexes=points, faces=faces )

    # 5. Tworzymy gotowy obiekt 3D
    mesh_item = gl.GLMeshItem(
        meshdata=mesh_data,
        smooth=True,         # Wygładzanie krawędzi (jak w prawdziwym CAD)
        color=color,
        #drawEdges = True,
        shader='balloon'     # Nasz shader z cieniami, żeby nie było czarne!
    )
    
    return mesh_item
# =====================================================================
# 1. MODUŁ KINEMATYKI I WĄTKU (Bez zmian - logika biznesowa)
# =====================================================================
class ScaraKinematics:
    def __init__(self):
        self.l1, self.r1 = 200.0, 200.0
        self.l2, self.r2 = 250.0, 250.0
        self.r = 100.0
        
    def get_plot_points(self, Arad: float, Brad: float):
        Xa_base, Ya_base = -self.r / 2, 0
        Xb_base, Yb_base = self.r / 2, 0
        Xa_elbow = self.l1 * math.cos(Arad) + Xa_base
        Ya_elbow = self.l1 * math.sin(Arad) + Ya_base
        Xb_elbow = self.r1 * math.cos(Brad) + Xb_base
        Yb_elbow = self.r1 * math.sin(Brad) + Yb_base

        psi = math.atan2((Yb_elbow - Ya_elbow), (Xb_elbow - Xa_elbow))
        d = math.sqrt((Xb_elbow - Xa_elbow)**2 + (Yb_elbow - Ya_elbow)**2)
        if d > (self.l2 + self.r2) or d < abs(self.l2 - self.r2): return None

        delt = math.acos(d / (2 * self.l2))
        fi1 = psi + delt
        Xc = Xa_elbow + self.l2 * math.cos(fi1)
        Yc = Ya_elbow + self.l2 * math.sin(fi1)

        return {
            'baseA': (Xa_base, Ya_base), 'baseB': (Xb_base, Yb_base),
            'elbowA': (Xa_elbow, Ya_elbow), 'elbowB': (Xb_elbow, Yb_elbow),
            'tool': (Xc, Yc),
            'ang_L2': math.atan2(Yc - Ya_elbow, Xc - Xa_elbow),
            'ang_R2': math.atan2(Yc - Yb_elbow, Xc - Xb_elbow)
        }

class RobotDataThread(QThread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.auto_mode = False
        
        # Zmienne, które będą "podglądane" przez QTimer
        self.A_deg, self.B_deg, self.Z_pos = 45.0, 135.0, 50.0
        self.t = 0.0

    def run(self):
        while self.running:
            if self.auto_mode:
                self.A_deg = 90 + 30 * math.sin(self.t)
                self.B_deg = 90 - 30 * math.cos(self.t * 0.8)
                self.Z_pos = 50 + 40 * math.sin(self.t * 2)
                self.t += 0.03
                
            self.msleep(10) # Działa bardzo szybko (100 Hz), gada ze sprzętem

    def set_manual_pos(self, a, b, z):
        if not self.auto_mode:
            self.A_deg, self.B_deg, self.Z_pos = a, b, z

# =====================================================================
# 2. MODUŁY UI (Nasze nowe, inteligentne "Czarne Skrzynki")
# =====================================================================

class RealTimePlot(QWidget):
    """
    MODUŁ: Ekstremalnie wydajny wykres obsługujący WIELE ścieżek.
    Zoptymalizowany przez wyłączenie Auto-Range i ręczne przesuwanie osi X.
    """
    def __init__(self, title, y_label, max_history=100):
        super().__init__()
        self.max_history = max_history
        
        # Używamy zwykłych list - brak konieczności rzutowania przy rysowaniu
        self.czas_x = []
        self.dane_y = {}    
        self.curves = {}    

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.label = QLabel(f"<b>{title}</b>")
        layout.addWidget(self.label)

        self.plot_widget = pg.PlotWidget(background='w')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('bottom', 'Czas [s]')
        self.plot_widget.setLabel('left', y_label)
        
        # --- KLUCZOWA OPTYMALIZACJA (Eliminacja lagów) ---
        # Wyłączamy automatyczne skalowanie osi X i blokujemy myszkę na tej osi, 
        # żeby nie walczyła z naszym automatycznym przewijaniem.
        self.plot_widget.setMouseEnabled(x=False, y=True) 
        self.plot_widget.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        
        self.plot_widget.addLegend(offset=(10, 10))  
        layout.addWidget(self.plot_widget)

    def add_curve(self, name, color):
        """Dodaje nową linię do tego samego wykresu"""
        self.dane_y[name] = []
        # antialias=False wygładza krawędzie, ale bez niego rysowanie jest 2x szybsze
        self.curves[name] = self.plot_widget.plot(name=name, pen=pg.mkPen(color, width=2), antialias=False)

    def update_data(self, time_val, data_dict):
        """ Aktualizuje wszystkie ścieżki i płynnie przesuwa wykres """
        self.czas_x.append(time_val)
        
        for name, val in data_dict.items():
            if name in self.dane_y:
                self.dane_y[name].append(val)

        # Utrzymujemy stałą długość list (O(1) dla małych N)
        if len(self.czas_x) > self.max_history:
            self.czas_x.pop(0)
            for name in self.dane_y:
                self.dane_y[name].pop(0)

        # --- RĘCZNE PRZESUWANIE WYKRESU (Błyskawiczne) ---
        # Obliczamy szerokość "okna" (100 klatek * 33ms zakłada ok 3.3 sekundy w pamięci)
        szerokosc_okna = self.max_history * 0.033 
        min_x = max(0, time_val - szerokosc_okna)
        
        # Sztywno narzucamy wykresowi co ma pokazać
        self.plot_widget.setXRange(min_x, max(time_val, szerokosc_okna), padding=0)

        # Rysujemy WSZYSTKIE klatki - prawdziwe 30 FPS
        for name, curve in self.curves.items():
            curve.setData(self.czas_x, self.dane_y[name])
        
    def clear_data(self):
        self.czas_x.clear()
        for name in self.dane_y:
            self.dane_y[name].clear()
            self.curves[name].setData([], [])

# =====================================================================
# 3. GŁÓWNA APLIKACJA (Składa moduły w jedną całość)
# =====================================================================

class RobotVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.kin = ScaraKinematics()
        self.arm_w, self.arm_t, self.joint_r = 30, 16, 20
        self.start_time = time.time()
        
        self.initUI()
        self.build_3d_robot()
        
        # Uruchomienie wątku obliczeniowego (100 Hz w tle)
        self.thread = RobotDataThread()
        self.thread.start()

        # --- ZEGAR GUI (Nasz QTimer ustalający FPS) ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.redraw_gui)
        # ZMIANA: 33 ms = ~30 FPS (Optymalne dla GUI). Maksymalnie daj tu 16 (60 FPS)
        self.timer.start(16) 

    def initUI(self):
        self.setWindowTitle("Modułowy Symulator SCARA")
        self.resize(1400, 800)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # --- PANEL LEWY: 3D ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0) # Usuwa brzydkie marginesy

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=800, elevation=60, azimuth=135)
        self.view.setBackgroundColor('w')
        grid = gl.GLGridItem()
        grid.scale(50, 50, 1)
        grid.setColor('k')
        self.view.addItem(grid)
        left_layout.addWidget(self.view, stretch=4)

        # --- PANEL DOLNY ---
        self.bottom_panel = QWidget()
        self.bottom_panel.setStyleSheet("background-color: #f0f0f0; border-radius: 5px;") # Lekki kolorek dla odróżnienia
        self.bottom_layout = QHBoxLayout(self.bottom_panel)

        # Dodaję tu dwa przykładowe przyciski z Twojego szkicu, żebyś od razu widział efekt
        btn_test1 = QPushButton("Test 1")
        btn_test2 = QPushButton("Test 2")
        self.bottom_layout.addWidget(btn_test1)
        self.bottom_layout.addWidget(btn_test2)
        self.bottom_layout.addStretch() # Wypycha przyciski do lewej krawędzi
        
        left_layout.addWidget(self.bottom_panel, stretch=1) # Panel zajmuje 20% wysokości lewej strony

        main_layout.addWidget(left_panel, stretch=3)

        # --- PANEL PRAWY: Wykresy i Sterowanie ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, stretch=1)

        # 1. Moduł Sterowania
        control_group = QGroupBox("Panel Sterowania")
        control_layout = QVBoxLayout()
        
        self.check_auto = QCheckBox("Symulacja Live")
        self.check_auto.stateChanged.connect(lambda s: setattr(self.thread, 'auto_mode', s == Qt.Checked))
        control_layout.addWidget(self.check_auto)

        self.slider_a = self._make_slider(control_layout, "Kąt L1 (A)", -90, 270, 45)
        self.slider_b = self._make_slider(control_layout, "Kąt R1 (B)", -90, 270, 135)
        self.slider_z = self._make_slider(control_layout, "Oś Z", 0, 150, 50)
        
        btn_reset = QPushButton("Wyczyść Wykresy")
        btn_reset.clicked.connect(self.clear_all_plots)
        control_layout.addWidget(btn_reset)
        
        control_group.setLayout(control_layout)
        right_layout.addWidget(control_group)

        # 2. Moduły Wykresów
        # Wykres 1: Oba kąty razem
        self.plot_angles = RealTimePlot("Kąty Silników", "Kąt [°]")
        self.plot_angles.add_curve("Oś A (Lewa)", color='b')  
        self.plot_angles.add_curve("Oś B (Prawa)", color='r') 
        
        # Wykres 2: Tylko Oś Z
        self.plot_z = RealTimePlot("Oś Z (Chwytak)", "Wysokość [mm]")
        self.plot_z.add_curve("Wysokość Z", color='g')        
        
        right_layout.addWidget(self.plot_angles)
        right_layout.addWidget(self.plot_z)

    def _make_slider(self, layout, name, min_v, max_v, val):
        layout.addWidget(QLabel(name))
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_v, max_v)
        slider.setValue(val)
        slider.valueChanged.connect(self.on_slider_change)
        layout.addWidget(slider)
        return slider

    def on_slider_change(self):
        self.thread.set_manual_pos(self.slider_a.value(), self.slider_b.value(), self.slider_z.value())

    def clear_all_plots(self):
        # Poprawiona funkcja czyszcząca - teraz odwołuje się do poprawnych nowych wykresów
        self.plot_angles.clear_data()
        self.plot_z.clear_data()

    # --- PĘTLA GŁÓWNA ---
    def redraw_gui(self):
        a = self.thread.A_deg
        b = self.thread.B_deg
        z = self.thread.Z_pos

        # 1. Aktualizacja modelu 3D
        self.update_3d_view(a, b, z)
        
        # 2. Aktualizacja wykresów
        obecny_czas = time.time() - self.start_time
        
        self.plot_angles.update_data(obecny_czas, {
            "Oś A (Lewa)": a, 
            "Oś B (Prawa)": b
        })
        
        self.plot_z.update_data(obecny_czas, {
            "Wysokość Z": z
        })

    def build_3d_robot(self):
        wall_md = create_box_mesh(400, 20, 200)
        self.wall = gl.GLMeshItem(meshdata=wall_md, color=(0.4, 0.4, 0.45, 1), smooth=False)
        self.wall.translate(-200, -50, 100) 
        self.view.addItem(self.wall)

        col_arm_prox = (0.2, 0.5, 0.8, 1) 
        col_arm_dist = (0.2, 0.7, 0.6, 1) 
        col_joint = (0.1, 0.1, 0.1, 1)    

        def add_arm(length, color):
            arm = gl.GLMeshItem(meshdata=create_box_mesh(length, self.arm_w, self.arm_t), color=color, smooth=False, shader='balloon', drawEdges=True, edgeColor=(0, 0, 0, 0.5))
            j1 = gl.GLMeshItem(meshdata=create_cylinder_mesh(self.joint_r, self.arm_t+2), color=col_joint, smooth=True, shader='balloon', drawEdges=True, edgeColor=(0, 0, 0, 0.5))
            j2 = gl.GLMeshItem(meshdata=create_cylinder_mesh(self.joint_r, self.arm_t+2), color=col_joint, smooth=True, shader='balloon', drawEdges=True, edgeColor=(0, 0, 0, 0.5))
            j2.translate(length, 0, 0)
            j1.setParentItem(arm) 
            j2.setParentItem(arm)
            self.view.addItem(arm)
            return arm

        CURRENT_DIR = pathlib.Path(__file__).parent
        stl_path = CURRENT_DIR / "assets" / "meshes" / "1.stl"
        self.arm_l1 = load_stl_mesh(str(stl_path), color=(0.2, 0.5, 0.8, 1))
        self.view.addItem(self.arm_l1)
        #self.arm_l1 = add_arm(self.kin.l1, col_arm_prox)
        self.arm_r1 = add_arm(self.kin.r1, col_arm_prox)
        self.arm_l2 = add_arm(self.kin.l2, col_arm_dist)
        self.arm_r2 = add_arm(self.kin.r2, col_arm_dist)

        tool_md = create_cylinder_mesh(self.joint_r * 0.8, 100)
        self.tool = gl.GLMeshItem(meshdata=tool_md, color=(0.9, 0.8, 0.2, 1), smooth=True)
        self.view.addItem(self.tool)

    def update_3d_view(self, a_deg, b_deg, z_pos):
        pts = self.kin.get_plot_points(math.radians(a_deg), math.radians(b_deg))
        if pts is None: return 

        z_layer1 = 150 
        z_layer2 = 150 + self.arm_t + 2 

        tr_l1 = pg.Transform3D()
        tr_l1.translate(pts['baseA'][0], pts['baseA'][1], z_layer1)
        tr_l1.rotate(a_deg, 0, 0, 1)
        tr_l1.rotate(90, 0, 0, 1) # korekta stl
        self.arm_l1.setTransform(tr_l1)

        tr_r1 = pg.Transform3D()
        tr_r1.translate(pts['baseB'][0], pts['baseB'][1], z_layer1)
        tr_r1.rotate(b_deg, 0, 0, 1)
        self.arm_r1.setTransform(tr_r1)

        tr_l2 = pg.Transform3D()
        tr_l2.translate(pts['elbowA'][0], pts['elbowA'][1], z_layer2)
        tr_l2.rotate(math.degrees(pts['ang_L2']), 0, 0, 1)
        self.arm_l2.setTransform(tr_l2)

        tr_r2 = pg.Transform3D()
        tr_r2.translate(pts['elbowB'][0], pts['elbowB'][1], z_layer2)
        tr_r2.rotate(math.degrees(pts['ang_R2']), 0, 0, 1)
        self.arm_r2.setTransform(tr_r2)

        tr_tool = pg.Transform3D()
        tr_tool.translate(pts['tool'][0], pts['tool'][1], z_layer2 - 50 + z_pos)
        self.tool.setTransform(tr_tool)

    def closeEvent(self, event):
        self.thread.running = False
        self.thread.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotVisualizer()
    window.show()
    sys.exit(app.exec_())