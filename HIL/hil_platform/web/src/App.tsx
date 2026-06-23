// 单前端项目，两个页面共用顶部导航 + 路由出口。
import { NavLink, Outlet } from "react-router-dom";

export default function App() {
  return (
    <div style={{ minHeight: "100%" }}>
      <nav className="topnav">
        <div className="brand">
          ADAS <span>HIL</span> 监控与回放平台
        </div>
        <NavLink to="/live" className={({ isActive }) => "navlink" + (isActive ? " active" : "")}>
          实时监控
        </NavLink>
        <NavLink to="/replay" className={({ isActive }) => "navlink" + (isActive ? " active" : "")}>
          历史回放
        </NavLink>
        <div className="spacer" />
        <span className="faint">CARLA + 双 Nano + ESP32 仲裁 · HIL 闭环</span>
      </nav>
      <Outlet />
    </div>
  );
}
