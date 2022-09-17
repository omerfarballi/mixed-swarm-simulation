import time
import demo
import ipc

# Demo için bazı kontrolleri şimdilik devre dışı bırakalım
ipc.CmdDebugStates.set_ignore_crashes(True)
ipc.CmdDebugStates.set_ignore_lifetime(True)
ipc.CmdDebugStates.set_ignore_speed_limit(True)


# Sonsuz döngü altında devamlı şimdiki durumu okuyup geleceğe yönelik karar verme yöntemini takip ediniz.
while True:
    ipc.perform_heartbeat()  # Oyunun çalışması için devamlı kalp atışı gönderilmelidir
    demo.main_loop()         # Kodunuz sonsuz döngü altında mevcut state'i okuyup yeni komutlar gönderir
    time.sleep(0.05)         # Uygun bir sleep koyarak programın kilitlenmesini önleyin
