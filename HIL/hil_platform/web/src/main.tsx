import React from "react";
import ReactDOM from "react-dom/client";
import { createBrowserRouter, RouterProvider, Navigate } from "react-router-dom";
import App from "./App";
import LivePage from "./pages/LivePage";
import ReplayPage from "./pages/ReplayPage";
import CoreLayoutPage from "./pages/CoreLayoutPage";
import LoongsonCloudPage from "./pages/LoongsonCloudPage";
import "./index.css";

const router = createBrowserRouter([
  {
    path: "/",
    element: <App />,
    children: [
      { index: true, element: <Navigate to="/live" replace /> },
      { path: "live", element: <LivePage /> },
      { path: "replay", element: <ReplayPage /> },
      { path: "cores", element: <CoreLayoutPage /> },
      { path: "loongson-cloud", element: <LoongsonCloudPage /> },
    ],
  },
]);

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>
);
