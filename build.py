import subprocess
import os
import shutil

def run_command(cmd, cwd, extra_env=None):
    print(f"--- Running: {cmd} in {cwd} ---")
    
    # Копируем текущие переменные окружения системы
    current_env = os.environ.copy()
    # Если мы передали дополнительные настройки, объединяем их
    if extra_env:
        current_env.update(extra_env)
    
    # Передаем current_env в параметр env
    result = subprocess.run(cmd, shell=True, cwd=cwd, env=current_env)
    if result.returncode != 0:
        print(f"Error executing command in {cwd}")
        exit(1)

# 1. Сборка CH32 (вызовом cmake)
ch32_dir = "./slave_controller"
ch32_build_dir = os.path.join(ch32_dir, "build_slave")
run_command("cmake -B build_slave -G \"MinGW Makefiles\"", ch32_dir)
run_command("cmake --build build_slave", ch32_dir)
# Опционально: Копируем бинарник CH32 в папку
#ch_bin_path = os.path.join(ch32_build_dir, "stm32_firmware.bin")
#esp_embed_path = "./output/slave_fw.bin"
#if os.path.exists(ch_bin_path):
#    shutil.copy(ch_bin_path, esp_embed_path)
#    print(f"Copied CH32 firmware to {esp_embed_path}")
# 2. Подготовка окружения для ESP32
# Добавляем путь к скриптам Python в начало PATH
esp_python_scripts = r"F:\work\esp32\esp-idf\python_env\idf5.5_py3.11_env\Scripts"
new_path = f"{esp_python_scripts};{os.environ.get('PATH', '')}"

# Для ESP-IDF важно не просто изменить PATH, но и выполнить экспорт.
# В Python проще всего вызвать idf.py через цепочку команд в одной сессии:
esp_dir = "./"
export_script = r'F:\work\esp32\esp-idf\frameworks\esp-idf-v5.5.2\export.ps1'

# Команда для PowerShell: сначала запускаем экспорт, затем сборку
# Используем . (dot sourcing), чтобы переменные остались в сессии для idf.py
esp_cmd = f'powershell -Command ". {export_script}; idf.py build flash"'

run_command(esp_cmd, esp_dir, extra_env={"PATH": new_path})


print("\n--- All projects built successfully! ---")