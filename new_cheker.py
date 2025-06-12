import cv2
import numpy as np
import os
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas

def compute_blur(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def compute_metrics(ref, test):
    psnr_val = cv2.PSNR(ref, test)
    ssim_val = ssim(ref, test, channel_axis=2)
    return psnr_val, ssim_val

def highlight_distortion_edges(reference, target):
    gray_ref = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    gray_target = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray_ref, gray_target)
    edges = cv2.Canny(diff, 50, 150)
    overlay = target.copy()
    overlay[edges > 0] = [0, 255, 0]  # green lines where distortion remains
    return overlay

def draw_frame(frame_in, frame1, frame2, text1, text2):
    combined = np.hstack([frame_in, frame1, frame2])
    h, w = frame1.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(combined, "Input", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(combined, "Output 1", (w + 10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(combined, "Output 2", (2*w + 10, 30), font, 1, (0, 255, 0), 2)
    cv2.putText(combined, text1, (w + 10, h - 10), font, 0.6, (255, 255, 255), 1)
    cv2.putText(combined, text2, (2*w + 10, h - 10), font, 0.6, (255, 255, 255), 1)
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

def analyze(distorted, output1, output2, max_frames=500):
    cap_in = cv2.VideoCapture(distorted)
    cap1 = cv2.VideoCapture(output1)
    cap2 = cv2.VideoCapture(output2)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_vid = cv2.VideoWriter('comparison_output.mp4', fourcc, 30.0,
                              (int(cap_in.get(3))*3, int(cap_in.get(4))))

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

        f1_highlight = highlight_distortion_edges(fin, f1)
        f2_highlight = highlight_distortion_edges(fin, f2)

        out_frame = draw_frame(fin, f1_highlight, f2_highlight, text1, text2)
        out_vid.write(out_frame)

        i += 1

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
    undistorted1 = "/home/amither/Downloads/calibartion/build_amd/output.mp4"
    undistorted2 = "/home/amither/Downloads/2025-05-13_14-55-16_undist.mp4"

    if not all(os.path.exists(f) for f in [distorted_video, undistorted1, undistorted2]):
        print("❌ One or more video paths are invalid. Please check the filenames.")
    else:
        analyze(distorted_video, undistorted1, undistorted2)