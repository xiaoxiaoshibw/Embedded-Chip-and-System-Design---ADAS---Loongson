// 轻量 ECharts 封装：只按需引入图表/组件，减小打包体积。
import { useEffect, useRef } from "react";
import * as echarts from "echarts/core";
import { LineChart } from "echarts/charts";
import {
  GridComponent, TooltipComponent, LegendComponent,
  MarkLineComponent, MarkPointComponent, DataZoomComponent,
} from "echarts/components";
import { CanvasRenderer } from "echarts/renderers";

echarts.use([
  LineChart, GridComponent, TooltipComponent, LegendComponent,
  MarkLineComponent, MarkPointComponent, DataZoomComponent, CanvasRenderer,
]);

interface Props {
  // 用 unknown 接收，内部再断言为 ECharts 选项，方便各页面传入字面量对象
  option: unknown;
  height?: number | string;
  onPointClick?: (dataIndex: number, seriesIndex: number) => void;
}

export function EChart({ option, height = 180, onPointClick }: Props) {
  const ref = useRef<HTMLDivElement | null>(null);
  const chartRef = useRef<echarts.ECharts | null>(null);

  useEffect(() => {
    if (!ref.current) return;
    const chart = echarts.init(ref.current, "dark", { renderer: "canvas" });
    chartRef.current = chart;
    const ro = new ResizeObserver(() => chart.resize());
    ro.observe(ref.current);
    return () => {
      ro.disconnect();
      chart.dispose();
      chartRef.current = null;
    };
  }, []);

  useEffect(() => {
    chartRef.current?.setOption(option as echarts.EChartsCoreOption, true);
  }, [option]);

  useEffect(() => {
    const chart = chartRef.current;
    if (!chart || !onPointClick) return;
    const handler = (p: { dataIndex: number; seriesIndex: number }) =>
      onPointClick(p.dataIndex, p.seriesIndex);
    chart.on("click", handler as never);
    return () => { chart.off("click", handler as never); };
  }, [onPointClick]);

  return <div ref={ref} style={{ width: "100%", height, background: "transparent" }} />;
}
