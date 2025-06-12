import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
import os

#Higher Blur variance → Sharper image
#Higher PSNR/SSIM → More faithful to input (if undistortion preserves content well)
#This tool helps you judge which pipeline gives better undistorted results.

# ✅ PSNR (Peak Signal-to-Noise Ratio)
# Measures pixel-level similarity between two images.
    # Higher = better.
    # Typically:
    # >30 dB = very good
    # 20–30 dB = moderate quality
    # <20 dB = noticeable degradation
    # It is sensitive to brightness/contrast differences, but doesn't model human perception well.

# ✅ SSIM (Structural Similarity Index)
# Measures perceptual quality: structure, luminance, contrast.
    # Range: 0 to 1
    # Higher = better.
    # Typically:
    # >0.95 = excellent quality
    # 0.85–0.95 = good
    # <0.85 = visible quality loss
# SSIM is generally more aligned with human visual perception than PSNR.

import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.metrics import structural_similarity as ssim
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
import os

def compute_blur(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def compute_metrics(ref, test):
    psnr_val = cv2.PSNR(ref, test)
    ssim_val = ssim(ref, test, channel_axis=2)
    return psnr_val, ssim_val

def draw_frame_with_overlay(frame_in, frame1, frame2, text1, text2, opacity=0.5):
    h, w = frame_in.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Ensure both frames are the same size
    frame1 = cv2.resize(frame1, (w, h))
    frame2 = cv2.resize(frame2, (w, h))

    # Create overlay: frame2 on top of frame1 with transparency
    blended = cv2.addWeighted(frame1, 1 - opacity, frame2, opacity, 0)

    # Draw red center cross
    def draw_cross(img):
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        cv2.line(img, (cx - 10, cy), (cx + 10, cy), (0, 0, 255), 2)
        cv2.line(img, (cx, cy - 10), (cx, cy + 10), (0, 0, 255), 2)
        return img

    frame_in = draw_cross(frame_in.copy())
    blended = draw_cross(blended.copy())

    # Stack layout: Input | Overlay
    combined = np.hstack([frame_in, blended])

    # Add text
    cv2.putText(combined, "Input", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(combined, "Overlay: Output2 on Output1", (w + 10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(combined, text1, (w + 10, h - 40), font, 0.6, (0, 255, 0), 1)
    cv2.putText(combined, text2, (w + 10, h - 20), font, 0.6, (0, 255, 0), 1)

    return combined

def draw_frame(frame_in, frame1, frame2, text1, text2):
    h, w = frame_in.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Blended overlay view (helps visually check distortions)
    blend1 = cv2.addWeighted(frame_in, 0.5, frame1, 0.5, 0)
    blend2 = cv2.addWeighted(frame_in, 0.5, frame2, 0.5, 0)

    # Draw red cross at the center
    def draw_cross(img):
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        cv2.line(img, (cx - 10, cy), (cx + 10, cy), (0, 0, 255), 2)
        cv2.line(img, (cx, cy - 10), (cx, cy + 10), (0, 0, 255), 2)
        return img

    frame_in = draw_cross(frame_in.copy())
    frame1 = draw_cross(frame1.copy())
    frame2 = draw_cross(frame2.copy())
    blend1 = draw_cross(blend1.copy())
    blend2 = draw_cross(blend2.copy())

    # Stack layout: Original | Output1 | Output2
    row1 = np.hstack([frame_in, frame1, frame2])
    row2 = np.hstack([np.zeros_like(frame_in), blend1, blend2])  # Empty input slot

    combined = np.vstack([row1, row2])

    # Label top row
    cv2.putText(combined, "Input", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(combined, "Output 1", (w + 10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(combined, "Output 2", (2 * w + 10, 30), font, 1, (0, 255, 0), 2)

    # Add metric text below the respective outputs
    cv2.putText(combined, text1, (w + 10, h * 2 - 20), font, 0.6, (255, 255, 255), 1)
    cv2.putText(combined, text2, (2 * w + 10, h * 2 - 20), font, 0.6, (255, 255, 255), 1)

    return combined

def plot_metrics(psnrs1, psnrs2, ssims1, ssims2, blurs1, blurs2):
    fig, axs = plt.subplots(3, 1, figsize=(10, 10))
    axs[0].plot(psnrs1, label='PSNR Output 1')
    axs[0].plot(psnrs2, label='PSNR Output 2')
    axs[0].legend(); axs[0].set_title("PSNR")

    axs[1].plot(ssims1, label='SSIM Output 1')
    axs[1].plot(ssims2, label='SSIM Output 2')
    axs[1].legend(); axs[1].set_title("SSIM")

    axs[2].plot(blurs1, label='Blur Output 1')
    axs[2].plot(blurs2, label='Blur Output 2')
    axs[2].legend(); axs[2].set_title("Blur (Laplacian Variance)")

    plt.tight_layout()
    plt.savefig("undistortion_metrics_plot.png")  # Save as image
    plt.close()

def generate_pdf_report(avg1, avg2):
    c = canvas.Canvas("undistortion_comparison_report.pdf", pagesize=letter)
    text = c.beginText(50, 750)
    text.setFont("Helvetica", 12)

    text.textLine("Undistortion Quality Report")
    text.textLine("")
    text.textLine("=== Output 1 ===")
    for k, v in avg1.items():
        text.textLine(f"{k}: {v:.4f}")
    text.textLine("")
    text.textLine("=== Output 2 ===")
    for k, v in avg2.items():
        text.textLine(f"{k}: {v:.4f}")

    winner = "Output 1" if avg1["SSIM"] > avg2["SSIM"] and avg1["Blur"] > avg2["Blur"] else "Output 2"
    text.textLine("")
    text.textLine(f"→ Better quality detected: {winner}")

    c.drawText(text)
    c.drawImage("undistortion_metrics_plot.png", 50, 300, width=500, height=400)
    c.save()

def analyze(distorted, output1, output2, max_frames=400):
    cap_in = cv2.VideoCapture(distorted)
    cap1 = cv2.VideoCapture(output1)
    cap2 = cv2.VideoCapture(output2)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    width = int(cap_in.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap_in.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out_vid = cv2.VideoWriter('comparison_output.mp4', fourcc, 30.0, (width * 2, height))


    psnrs1, psnrs2 = [], []
    ssims1, ssims2 = [], []
    blurs1, blurs2 = [], []

    i = 0
    while i < max_frames:
        ret_in, fin = cap_in.read()
        ret1, f1 = cap1.read()
        ret2, f2 = cap2.read()
        if not (ret_in and ret1 and ret2): break

        f1 = cv2.resize(f1, (fin.shape[1], fin.shape[0]))
        f2 = cv2.resize(f2, (fin.shape[1], fin.shape[0]))

        blur1 = compute_blur(cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY))
        blur2 = compute_blur(cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY))
        psnr1, ssim1_val = compute_metrics(fin, f1)
        psnr2, ssim2_val = compute_metrics(fin, f2)

        blurs1.append(blur1); blurs2.append(blur2)
        psnrs1.append(psnr1); psnrs2.append(psnr2)
        ssims1.append(ssim1_val); ssims2.append(ssim2_val)

        text1 = f"PSNR: {psnr1:.1f}  SSIM: {ssim1_val:.3f}  Blur: {blur1:.1f}"
        text2 = f"PSNR: {psnr2:.1f}  SSIM: {ssim2_val:.3f}  Blur: {blur2:.1f}"
        out_frame = draw_frame_with_overlay(fin, f1, f2, text1, text2, opacity=0.5)
        out_vid.write(out_frame)

        i += 1

        if ((i % 10) == 0):
            print(f"Progress - {i}")

    cap_in.release(); cap1.release(); cap2.release(); out_vid.release()

    avg1 = {
        "PSNR": np.mean(psnrs1),
        "SSIM": np.mean(ssims1),
        "Blur": np.mean(blurs1)
    }
    avg2 = {
        "PSNR": np.mean(psnrs2),
        "SSIM": np.mean(ssims2),
        "Blur": np.mean(blurs2)
    }

    plot_metrics(psnrs1, psnrs2, ssims1, ssims2, blurs1, blurs2)
    generate_pdf_report(avg1, avg2)

    print("\n✅ Analysis complete. Files generated:")
    print("- comparison_output.mp4")
    print("- undistortion_metrics_plot.pdf")
    print("- undistortion_comparison_report.pdf")
    
if __name__ == "__main__":
    distorted_video = "/home/amither/Downloads/2025-05-13_14-55-16.mp4"
    undistorted1 = "/home/amither/Downloads/calibartion/build_amd/output_bi.mp4"
    undistorted2 = "/home/amither/Downloads/2025-05-13_14-55-16_undist.mp4"

    if not all(os.path.exists(f) for f in [distorted_video, undistorted1, undistorted2]):
        print("❌ One or more video paths are invalid. Please check the filenames.")
    else:
        analyze(distorted_video, undistorted1, undistorted2)
