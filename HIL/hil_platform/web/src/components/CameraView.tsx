// CARLA RGB 摄像头实时画面（轮询 PNG，零额外依赖）。
// mock 模式或 CARLA 未就绪时显示占位提示，不报错。
import { useEffect, useRef, useState } from "react";
import { cameraUrl } from "../api/client";

export function CameraView({ active }: { active: boolean }) {
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [ok, setOk] = useState(false);

  useEffect(() => {
    if (!active) return;
    const id = window.setInterval(() => {
      if (imgRef.current) imgRef.current.src = cameraUrl();
    }, 200);   // ~5Hz
    return () => window.clearInterval(id);
  }, [active]);

  return (
    <div className="card" style={{ padding: 8 }}>
      <h3>CARLA 实时画面</h3>
      <div style={{ position: "relative", background: "#000", borderRadius: 6, minHeight: 200,
        display: "flex", alignItems: "center", justifyContent: "center" }}>
        <img
          ref={imgRef}
          alt="carla camera"
          onLoad={() => setOk(true)}
          onError={() => setOk(false)}
          style={{ width: "100%", borderRadius: 6, display: ok ? "block" : "none" }}
        />
        {!ok && (
          <span className="faint" style={{ padding: 24, textAlign: "center" }}>
            摄像头未就绪（mock 模式，或 CARLA 尚未加载场景）。<br />
            真实模式下加载场景并开始后此处显示车载视角。
          </span>
        )}
      </div>
    </div>
  );
}
