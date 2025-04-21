import sys
import glob
import serial
import pyautogui
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

def move_mouse(roll: float, pitch: float, sensitivity: float = 1.0):
    """
    Move o mouse: roll → X, pitch → Y.
    sensitivity é um multiplicador para ajustar a velocidade.
    """
    dx = roll * sensitivity
    dy = pitch * sensitivity
    pyautogui.moveRel(dx, dy)

def serial_reader(ser):
    """
    Thread que lê linhas CSV da porta serial:
    roll,pitch,yaw,click
    """
    prev_click = 0
    sensitivity = 1.0  # ajuste de acordo com o quanto quer movimentar
    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 4:
                # linha "mimada"?
                continue

            roll, pitch, yaw = map(float, parts[:3])
            click = int(parts[3])

            # move o mouse
            move_mouse(roll, pitch, sensitivity)

            # click na transição 0→1
            if click and not prev_click:
                pyautogui.click()

            prev_click = click

        except Exception as e:
            print("Erro na leitura/parsing:", e)

def serial_ports():
    """Detecta portas seriais disponíveis."""
    ports = []
    if sys.platform.startswith('win'):
        candidates = [f'COM{i}' for i in range(1, 256)]
    elif sys.platform.startswith(('linux', 'cygwin')):
        candidates = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        candidates = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada.')

    for port in candidates:
        try:
            s = serial.Serial(port)
            s.close()
            ports.append(port)
        except (OSError, serial.SerialException):
            pass

    return ports

def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    """Abre a porta e dispara thread de leitura."""
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta antes de conectar.")
        return

    try:
        ser = serial.Serial(port_name, 115200, timeout=0.1)
        status_label.config(text=f"Conectado em {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(state="disabled")

        # inicia a leitura em background
        t = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
        t.start()

    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}:\n{e}")
        mudar_cor_circulo("red")

def criar_janela():
    root = tk.Tk()
    root.title("Controle de Mouse")
    root.geometry("400x250")
    root.resizable(False, False)

    dark_bg = "#2e2e2e"
    dark_fg = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background=dark_bg)
    style.configure("TLabel", background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
    style.configure("TButton", font=("Segoe UI", 10, "bold"),
                    foreground=dark_fg, background="#444444", borderwidth=0)
    style.map("TButton", background=[("active", "#555555")])
    style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"),
                    foreground=dark_fg, background=accent_color, padding=6)
    style.map("Accent.TButton", background=[("active", "#005f9e")])
    style.configure("TCombobox",
                    fieldbackground=dark_bg,
                    background=dark_bg,
                    foreground=dark_fg,
                    padding=4)
    style.map("TCombobox", fieldbackground=[("readonly", dark_bg)])

    frame = ttk.Frame(root, padding="20")
    frame.pack(expand=True, fill="both")

    ttk.Label(frame, text="Controle de Mouse", font=("Segoe UI", 14, "bold")).pack(pady=(0,10))

    porta_var = tk.StringVar()
    botao_conectar = ttk.Button(frame, text="Conectar e Iniciar", style="Accent.TButton",
                                command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo))
    botao_conectar.pack(pady=10)

    footer = tk.Frame(root, bg=dark_bg)
    footer.pack(side="bottom", fill="x", padx=10, pady=(10,0))
    status_label = tk.Label(footer, text="Aguardando seleção de porta...", font=("Segoe UI",11),
                            bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    portas = serial_ports()
    if portas:
        porta_var.set(portas[0])
    port_dropdown = ttk.Combobox(footer, textvariable=porta_var,
                                 values=portas, state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle = tk.Canvas(footer, width=20, height=20, highlightthickness=0, bg=dark_bg)
    circle_item = circle.create_oval(2,2,18,18, fill="red", outline="")
    circle.grid(row=0, column=2, sticky="e")
    footer.columnconfigure(1, weight=1)

    def mudar_cor_circulo(cor):
        circle.itemconfig(circle_item, fill=cor)

    root.mainloop()

if __name__ == "__main__":
    criar_janela()
