import qrcode
from fpdf import FPDF
import os

# CONFIGURAZIONE
NUM_QR = 5            # Quanti QR generare (ID 0, 1, 2, 3, 4)
FILENAME = "QR_Codes_SLAM.pdf"

# Crea il PDF (A4 = 210mm x 297mm)
pdf = FPDF(orientation='P', unit='mm', format='A4')
pdf.set_auto_page_break(auto=False)

print(f"Generazione di {NUM_QR} QR Code in corso...")

for qr_id in range(NUM_QR):
    pdf.add_page()
    
    # 1. Genera l'immagine QR
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H, # Alta correzione errori
        box_size=10,
        border=4,  # QUIET ZONE (4 è lo standard minimo, lascialo così!)
    )
    qr.add_data(str(qr_id)) # Il dato è semplicemente il numero (es. "0")
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    
    # Salva temporaneamente l'immagine
    temp_img = f"temp_qr_{qr_id}.png"
    img.save(temp_img)
    
    # 2. Inserisci nel PDF
    # Un A4 è largo 210mm. 
    # Se vogliamo 15cm (150mm) "da nero a nero", con il bordo l'immagine totale sarà circa 180mm.
    pdf.image(temp_img, x=15, y=50, w=180) 
    
    # 3. Aggiungi testo per l'uomo (ID scritto sotto)
    pdf.set_font("Arial", size=24)
    pdf.set_xy(0, 240)
    pdf.cell(210, 10, txt=f"Landmark ID: {qr_id}", align='C')
    
    # Rimuovi file temporaneo
    os.remove(temp_img)

pdf.output(FILENAME)
print(f"✅ Fatto! Apri il file: {FILENAME}")