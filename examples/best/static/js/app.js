// 迷宫SLAM实时可视化系统 - 前端应用程序

class MazeSLAMApp {
    constructor() {
        this.socket = null;
        this.isConnected = false;
        this.simulationRunning = false;
        this.chartData = {
            trueMaze: null,
            slamMap: null,
            frontier: null,
            radar: null
        };
        
        // 初始化应用
        this.init();
    }
    
    init() {
        console.log('🚀 初始化迷宫SLAM可视化系统...');
        
        // 初始化WebSocket连接
        this.initWebSocket();
        
        // 初始化事件监听器
        this.initEventListeners();
        
        // 初始化图表
        this.initCharts();
        
        // 重置参数
        this.resetParameters();
        
        console.log('✅ 系统初始化完成');
    }
    
    initWebSocket() {
        // 连接到WebSocket服务器
        this.socket = io();
        
        // 连接事件
        this.socket.on('connect', () => {
            console.log('🔗 WebSocket连接成功');
            this.isConnected = true;
            this.updateConnectionStatus(true);
            this.addLog('WebSocket连接成功', 'success');
        });
        
        // 断开连接事件
        this.socket.on('disconnect', () => {
            console.log('❌ WebSocket连接断开');
            this.isConnected = false;
            this.updateConnectionStatus(false);
            this.addLog('WebSocket连接断开', 'error');
        });
        
        // 状态消息
        this.socket.on('status', (data) => {
            console.log('📡 状态消息:', data.message);
            this.addLog(data.message, 'info');
        });
        
        // 仿真更新
        this.socket.on('simulation_update', (data) => {
            this.handleSimulationUpdate(data);
        });
        
        // 仿真结束
        this.socket.on('simulation_ended', (data) => {
            console.log('🏁 仿真结束:', data.message);
            this.simulationRunning = false;
            this.updateSimulationStatus(false);
            this.addLog('仿真结束: ' + data.message, 'warning');
            this.hideLoadingOverlay();
        });
        
        // 错误处理
        this.socket.on('error', (data) => {
            console.error('❌ 仿真错误:', data.message);
            this.addLog('错误: ' + data.message, 'error');
            this.hideLoadingOverlay();
        });
    }
    
    initEventListeners() {
        // 开始仿真按钮
        document.getElementById('startBtn').addEventListener('click', () => {
            this.startSimulation();
        });
        
        // 停止仿真按钮
        document.getElementById('stopBtn').addEventListener('click', () => {
            this.stopSimulation();
        });
        
        // 重置参数按钮
        document.getElementById('resetBtn').addEventListener('click', () => {
            this.resetParameters();
        });
        
        // 模态框关闭
        document.getElementById('modalClose').addEventListener('click', () => {
            this.hideModal();
        });
        
        // 地图尺寸变化监听
        document.getElementById('mazeSize').addEventListener('input', (e) => {
            const size = e.target.value;
            document.querySelector('.unit').textContent = `x ${size}`;
        });
        
        // 窗口调整时重新布局图表
        window.addEventListener('resize', () => {
            this.resizeCharts();
        });
    }
    
    initCharts() {
        // 初始化所有图表
        this.initTrueMazeChart();
        this.initSlamMapChart();
        this.initFrontierChart();
        this.initRadarChart();
        
        console.log('📊 图表初始化完成');
    }
    
    initTrueMazeChart() {
        const layout = {
            title: {
                text: '真实迷宫环境',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)',
                x: 1,
                y: 1
            },
            hovermode: 'closest',
            autosize: true
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('trueMazeChart', [], layout, config);
    }
    
    initSlamMapChart() {
        const layout = {
            title: {
                text: 'SLAM构建地图',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            },
            autosize: true
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('slamMapChart', [], layout, config);
    }
    
    initFrontierChart() {
        const layout = {
            title: {
                text: '前沿探索区域',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            },
            autosize: true
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('frontierChart', [], layout, config);
    }
    
    initRadarChart() {
        const layout = {
            title: {
                text: '激光雷达扫描',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [0, 8],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            },
            autosize: true
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('radarChart', [], layout, config);
    }
    
    async startSimulation() {
        if (!this.isConnected) {
            this.addLog('错误: WebSocket未连接', 'error');
            return;
        }
        
        if (this.simulationRunning) {
            this.addLog('仿真已在运行中', 'warning');
            return;
        }
        
        try {
            // 显示加载覆盖层
            this.showLoadingOverlay();
            
            // 获取用户输入参数
            const params = this.getSimulationParameters();
            
            // 发送启动请求
            const response = await fetch('/api/start_simulation', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(params)
            });
            
            const result = await response.json();
            
            if (result.status === 'success') {
                this.simulationRunning = true;
                this.updateSimulationStatus(true);
                this.addLog('仿真启动成功', 'success');
                // 保持加载界面，等待第一个数据更新
            } else {
                this.addLog('仿真启动失败: ' + result.message, 'error');
                this.hideLoadingOverlay();
            }
            
        } catch (error) {
            console.error('启动仿真失败:', error);
            this.addLog('启动仿真失败: ' + error.message, 'error');
            this.hideLoadingOverlay();
        }
    }
    
    async stopSimulation() {
        if (!this.simulationRunning) {
            this.addLog('仿真未在运行', 'warning');
            return;
        }
        
        try {
            const response = await fetch('/api/stop_simulation', {
                method: 'POST'
            });
            
            const result = await response.json();
            
            if (result.status === 'success') {
                this.simulationRunning = false;
                this.updateSimulationStatus(false);
                this.addLog('仿真已停止', 'success');
            } else {
                this.addLog('停止仿真失败: ' + result.message, 'error');
            }
            
        } catch (error) {
            console.error('停止仿真失败:', error);
            this.addLog('停止仿真失败: ' + error.message, 'error');
        }
    }
    
    getSimulationParameters() {
        // 获取地图尺寸
        const mazeSize = parseFloat(document.getElementById('mazeSize').value);
        
        // 获取起点坐标
        const startX = parseFloat(document.getElementById('startX').value);
        const startY = parseFloat(document.getElementById('startY').value);
        
        // 获取墙壁配置
        let walls = [];
        const wallsConfig = document.getElementById('wallsConfig').value.trim();
        
        if (wallsConfig) {
            try {
                walls = JSON.parse(wallsConfig);
            } catch (error) {
                console.warn('墙壁配置JSON解析失败，使用默认配置');
                this.addLog('墙壁配置格式错误，使用默认配置', 'warning');
            }
        }
        
        return {
            maze_size: mazeSize,
            start_pos: [startX, startY],
            walls: walls
        };
    }
    
    resetParameters() {
        // 重置所有参数到默认值
        document.getElementById('mazeSize').value = 8;
        document.getElementById('startX').value = 1.0;
        document.getElementById('startY').value = 1.0;
        document.getElementById('wallsConfig').value = '';
        document.querySelector('.unit').textContent = 'x 8';
        
        // 清空状态信息
        this.clearStatusInfo();
        
        // 清空日志
        this.clearLog();
        
        this.addLog('参数已重置为默认值', 'info');
    }
    
    handleSimulationUpdate(data) {
        try {
            // 隐藏加载覆盖层（如果还在显示）
            this.hideLoadingOverlay();
            
            // 更新状态信息
            if (data.status) {
                this.updateStatusInfo(data.status);
            }
            
            // 处理可能需要转换的数据
            if (data.slam_map) {
                // 确保机器人位置和轨迹数据被传递给图表更新函数
                if (data.status && data.status.robot_pos) {
                    data.slam_map.robot_pos = data.status.robot_pos;
                }
                
                if (data.status && data.status.robot_path) {
                    data.slam_map.robot_path = data.status.robot_path;
                }
            }
            
            if (data.frontier) {
                // 确保机器人位置和轨迹数据也被传递给前沿图更新函数
                if (data.status && data.status.robot_pos) {
                    data.frontier.robot_pos = data.status.robot_pos;
                }
                
                if (data.status && data.status.robot_path) {
                    data.frontier.robot_path = data.status.robot_path;
                }
            }
            
            // 更新图表
            if (data.true_maze) {
                this.updateTrueMazeChart(data.true_maze);
            }
            
            if (data.slam_map) {
                this.updateSlamMapChart(data.slam_map);
            }
            
            if (data.frontier) {
                this.updateFrontierChart(data.frontier);
            }
            
            if (data.radar) {
                this.updateRadarChart(data.radar);
            }
            
            // 更新进度
            if (data.status) {
                this.updateProgress(data.status.coverage || 0);
            }
            
            // 检查仿真是否完成
            if (data.simulation_completed) {
                this.simulationRunning = false;
                this.updateSimulationStatus(false);
                this.addLog('仿真完成！机器人已找到出口', 'success');
            }
            
        } catch (error) {
            console.error('处理仿真更新失败:', error);
            this.addLog('处理仿真更新失败: ' + error.message, 'error');
        }
    }
    
    updateTrueMazeChart(data) {
        const traces = [];
        
        // 添加墙壁
        if (data.walls && data.walls.length > 0) {
            const wallX = [];
            const wallY = [];
            
            data.walls.forEach(wall => {
                wallX.push(wall.x1, wall.x2, null);
                wallY.push(wall.y1, wall.y2, null);
            });
            
            traces.push({
                x: wallX,
                y: wallY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#ffffff', width: 3 },
                name: '墙壁',
                hoverinfo: 'none'
            });
        }
        
        // 添加机器人路径
        if (data.robot_path && data.robot_path.length > 0) {
            const pathX = data.robot_path.map(p => p[0]);
            const pathY = data.robot_path.map(p => p[1]);
            
            traces.push({
                x: pathX,
                y: pathY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#00d4ff', width: 2, dash: 'dash' },
                name: '机器人路径',
                opacity: 0.8
            });
        }
        
        // 添加起点
        if (data.start_pos) {
            traces.push({
                x: [data.start_pos[0]],
                y: [data.start_pos[1]],
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#ff6b35', size: 12, symbol: 'star' },
                name: '起点'
            });
        }
        
        // 添加机器人当前位置
        if (data.robot_pos) {
            traces.push({
                x: [data.robot_pos[0]],
                y: [data.robot_pos[1]],
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#00d4ff', size: 15, symbol: 'circle' },
                name: '机器人'
            });
        }
        
        // 添加目标位置
        if (data.target_pos) {
            traces.push({
                x: [data.target_pos[0]],
                y: [data.target_pos[1]],
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#ffff00', size: 12, symbol: 'x' },
                name: '目标点'
            });
        }
        
        // 添加发现的出口
        if (data.discovered_exits && data.discovered_exits.length > 0) {
            const exitX = data.discovered_exits.map(e => e[0]);
            const exitY = data.discovered_exits.map(e => e[1]);
            
            traces.push({
                x: exitX,
                y: exitY,
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#ff3366', size: 14, symbol: 'square' },
                name: '发现出口'
            });
        }
        
        // 添加到达的出口
        if (data.reached_exits && data.reached_exits.length > 0) {
            const reachedX = data.reached_exits.map(e => e[0]);
            const reachedY = data.reached_exits.map(e => e[1]);
            
            traces.push({
                x: reachedX,
                y: reachedY,
                type: 'scatter',
                mode: 'markers',
                marker: { 
                    color: 'rgba(255, 51, 102, 0.3)', 
                    size: 25, 
                    symbol: 'circle',
                    line: { color: '#ff3366', width: 3 }
                },
                name: '到达出口'
            });
        }
        
        // 添加最短路径
        if (data.shortest_path && data.shortest_path.length > 0) {
            const pathX = data.shortest_path.map(p => p[0]);
            const pathY = data.shortest_path.map(p => p[1]);
            
            traces.push({
                x: pathX,
                y: pathY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#00ff88', width: 4 },
                name: '最优路径',
                opacity: 0.9
            });
        }
        
        // 完整的布局设置
        const layout = {
            title: {
                text: '真实迷宫环境',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [-1, data.maze_size + 1],
                scaleanchor: "y",  // 确保x轴和y轴比例一致
                scaleratio: 1
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [-1, data.maze_size + 1]
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)',
                x: 1,
                y: 1
            },
            hovermode: 'closest'
        };
        
        Plotly.react('trueMazeChart', traces, layout);
    }
    
    updateSlamMapChart(data) {
        const traces = [];
        
        // 创建SLAM地图热图
        if (data.map_array && data.map_array.length > 0) {
            // 转换地图数据：0=未知(灰), 1=自由(白), 2=障碍(黑)
            const colorscale = [
                [0, '#4a4a4a'],    // 未知区域 - 深灰
                [0.5, '#ffffff'],  // 自由空间 - 白色
                [1, '#000000']     // 障碍物 - 黑色
            ];
            
            // 计算热图的x和y轴范围
            const gridSize = data.grid_size || data.map_array.length;
            const resolution = data.resolution || 0.1;
            const displaySize = data.display_size || data.maze_size;
            
            traces.push({
                z: data.map_array,
                type: 'heatmap',
                colorscale: colorscale,
                showscale: false,
                hoverinfo: 'none',
                x0: 0,  // 从0开始
                dx: resolution,
                y0: 0,  // 从0开始
                dy: resolution
            });
        }
        
        // 添加到达的出口标记
        if (data.reached_exits && data.reached_exits.length > 0) {
            // 确保所有坐标为正值
            const reachedX = data.reached_exits.map(e => Math.max(0, e[0]));
            const reachedY = data.reached_exits.map(e => Math.max(0, e[1]));
            
            traces.push({
                x: reachedX,
                y: reachedY,
                type: 'scatter',
                mode: 'markers',
                marker: { 
                    color: 'rgba(255, 51, 102, 0.3)', 
                    size: 25, 
                    symbol: 'circle',
                    line: { color: '#ff3366', width: 3 }
                },
                name: '到达出口'
            });
        }
        
        // 添加小车轨迹（如果存在）
        if (data.robot_path && data.robot_path.length > 0) {
            // 确保所有坐标为正值
            const pathX = data.robot_path.map(p => Math.max(0, p[0]));
            const pathY = data.robot_path.map(p => Math.max(0, p[1]));
            
            traces.push({
                x: pathX,
                y: pathY,
                type: 'scatter',
                mode: 'lines',
                line: { 
                    color: '#00d4ff',
                    width: 3,
                    dash: 'solid'
                },
                name: '机器人轨迹'
            });
        }
        
        // 添加当前机器人位置
        if (data.robot_pos) {
            // 确保坐标为正值
            const robotX = Math.max(0, data.robot_pos[0]);
            const robotY = Math.max(0, data.robot_pos[1]);
            
            traces.push({
                x: [robotX],
                y: [robotY],
                type: 'scatter',
                mode: 'markers',
                marker: { 
                    color: '#00ff88',
                    size: 12,
                    symbol: 'circle',
                    line: { color: '#ffffff', width: 2 }
                },
                name: '机器人位置'
            });
        }
        
        // 计算适当的坐标范围，确保从0开始
        let xMin = 0, xMax = Math.max(8, data.maze_size || 8);
        let yMin = 0, yMax = Math.max(8, data.maze_size || 8);
        
        // 如果有机器人位置，确保其在视野中
        if (data.robot_pos) {
            const margin = 1; // 在机器人周围留出的空间
            xMax = Math.max(xMax, data.robot_pos[0] + margin);
            yMax = Math.max(yMax, data.robot_pos[1] + margin);
        }
        
        // 如果有轨迹，确保整个轨迹在视野中
        if (data.robot_path && data.robot_path.length > 0) {
            const pathXMax = Math.max(...data.robot_path.map(p => p[0]));
            const pathYMax = Math.max(...data.robot_path.map(p => p[1]));
            
            const margin = 1; // 在轨迹周围留出的空间
            xMax = Math.max(xMax, pathXMax + margin);
            yMax = Math.max(yMax, pathYMax + margin);
        }
        
        // 完整的布局设置
        const layout = {
            title: {
                text: 'SLAM构建地图',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [xMin, xMax],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [yMin, yMax],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            }
        };
        
        Plotly.react('slamMapChart', traces, layout);
        
        // 强制调整大小以确保内容可见
        Plotly.Plots.resize('slamMapChart');
    }
    
    updateFrontierChart(data) {
        const traces = [];
        
        // 创建SLAM地图背景
        if (data.map_array && data.map_array.length > 0) {
            const colorscale = [
                [0, '#4a4a4a'],
                [0.5, '#ffffff'],
                [1, '#000000']
            ];
            
            const gridSize = data.grid_size || data.map_array.length;
            const resolution = data.resolution || 0.1;
            const displaySize = data.display_size || data.maze_size || 8;
            
            traces.push({
                z: data.map_array,
                type: 'heatmap',
                colorscale: colorscale,
                showscale: false,
                hoverinfo: 'none',
                opacity: 0.6,
                x0: 0,
                dx: resolution,
                y0: 0,
                dy: resolution
            });
        }
        
        // 添加前沿点
        if (data.frontiers && data.frontiers.length > 0) {
            // 确保所有坐标为正值
            const frontierX = data.frontiers.map(f => Math.max(0, f[0]));
            const frontierY = data.frontiers.map(f => Math.max(0, f[1]));
            
            traces.push({
                x: frontierX,
                y: frontierY,
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#ff6b35', size: 8, symbol: 'star' },
                name: `前沿点 (${data.frontiers.length})`
            });
        }
        
        // 添加机器人轨迹
        if (data.robot_path && data.robot_path.length > 0) {
            // 确保所有坐标为正值
            const pathX = data.robot_path.map(p => Math.max(0, p[0]));
            const pathY = data.robot_path.map(p => Math.max(0, p[1]));
            
            traces.push({
                x: pathX,
                y: pathY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#00d4ff', width: 2, dash: 'solid' },
                name: '机器人轨迹'
            });
        }
        
        // 添加机器人位置
        if (data.robot_pos) {
            // 确保坐标为正值
            const robotX = Math.max(0, data.robot_pos[0]);
            const robotY = Math.max(0, data.robot_pos[1]);
            
            traces.push({
                x: [robotX],
                y: [robotY],
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#00ff88', size: 12, symbol: 'circle', line: { color: '#ffffff', width: 2 } },
                name: '机器人'
            });
        }
        
        // 计算适当的坐标范围，从0开始
        let xMin = 0, xMax = Math.max(8, data.maze_size || 12);
        let yMin = 0, yMax = Math.max(8, data.maze_size || 12);
        
        // 如果有机器人位置，确保其在视野中
        if (data.robot_pos) {
            const margin = 1; // 在机器人周围留出的空间
            xMax = Math.max(xMax, data.robot_pos[0] + margin);
            yMax = Math.max(yMax, data.robot_pos[1] + margin);
        }
        
        // 如果有轨迹，确保整个轨迹在视野中
        if (data.robot_path && data.robot_path.length > 0) {
            const pathXMax = Math.max(...data.robot_path.map(p => p[0]));
            const pathYMax = Math.max(...data.robot_path.map(p => p[1]));
            
            const margin = 1; // 在轨迹周围留出的空间
            xMax = Math.max(xMax, pathXMax + margin);
            yMax = Math.max(yMax, pathYMax + margin);
        }
        
        const mazeSize = data.maze_size || 8;
        const layout = {
            title: {
                text: '前沿探索区域',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [xMin, xMax],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [yMin, yMax],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            }
        };
        
        Plotly.react('frontierChart', traces, layout);
        
        // 强制调整大小以确保内容可见
        Plotly.Plots.resize('frontierChart');
    }
    
    updateRadarChart(data) {
        const traces = [];
        
        // 添加墙壁
        if (data.walls && data.walls.length > 0) {
            const wallX = [];
            const wallY = [];
            
            data.walls.forEach(wall => {
                wallX.push(wall.x1, wall.x2, null);
                wallY.push(wall.y1, wall.y2, null);
            });
            
            traces.push({
                x: wallX,
                y: wallY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#ffffff', width: 2 },
                name: '墙壁',
                hoverinfo: 'none'
            });
        }
        
        // 添加机器人位置
        if (data.robot_pos) {
            // 确保坐标为正值
            const robotX = Math.max(0, data.robot_pos[0]);
            const robotY = Math.max(0, data.robot_pos[1]);
            
            traces.push({
                x: [robotX],
                y: [robotY],
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#00d4ff', size: 15, symbol: 'circle' },
                name: '机器人'
            });
        }
        
        // 添加扫描的自由空间点
        if (data.scan_points && data.scan_points.length > 0) {
            // 确保所有坐标为正值
            const scanX = data.scan_points.map(p => Math.max(0, p[0]));
            const scanY = data.scan_points.map(p => Math.max(0, p[1]));
            
            traces.push({
                x: scanX,
                y: scanY,
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#00ff88', size: 3, opacity: 0.6 },
                name: '自由空间',
                hoverinfo: 'none'
            });
        }
        
        // 添加障碍物点
        if (data.obstacle_points && data.obstacle_points.length > 0) {
            // 确保所有坐标为正值
            const obsX = data.obstacle_points.map(p => Math.max(0, p[0]));
            const obsY = data.obstacle_points.map(p => Math.max(0, p[1]));
            
            traces.push({
                x: obsX,
                y: obsY,
                type: 'scatter',
                mode: 'markers',
                marker: { color: '#ff3366', size: 6, opacity: 0.8 },
                name: '障碍物'
            });
        }
        
        // 绘制激光扫描射线（如果有扫描角度和距离数据）
        if (data.robot_pos && data.scan_ranges && data.scan_angles) {
            const robotX = Math.max(0, data.robot_pos[0]);
            const robotY = Math.max(0, data.robot_pos[1]);
            
            // 选择几条代表性射线绘制
            const rayIndices = [];
            const totalRays = data.scan_angles.length;
            for (let i = 0; i < totalRays; i += Math.max(1, Math.floor(totalRays / 16))) {
                rayIndices.push(i);
            }
            
            const rayX = [];
            const rayY = [];
            
            rayIndices.forEach(i => {
                const angle = data.scan_angles[i];
                const range = data.scan_ranges[i];
                
                const endX = robotX + range * Math.cos(angle);
                const endY = robotY + range * Math.sin(angle);
                // 确保射线端点坐标为正值
                rayX.push(robotX, Math.max(0, endX), null);
                rayY.push(robotY, Math.max(0, endY), null);
            });
            
            traces.push({
                x: rayX,
                y: rayY,
                type: 'scatter',
                mode: 'lines',
                line: { color: '#ffff00', width: 1, dash: 'dot' },
                name: '激光射线',
                opacity: 0.3,
                hoverinfo: 'none'
            });
        }
        
        // 计算适当的坐标范围，从0开始
        let xMin = 0, xMax = Math.max(8, data.maze_size || 12);
        let yMin = 0, yMax = Math.max(8, data.maze_size || 12);
        
        // 如果有机器人位置，确保其在视野中
        if (data.robot_pos) {
            const margin = 1; // 在机器人周围留出的空间
            xMax = Math.max(xMax, data.robot_pos[0] + margin);
            yMax = Math.max(yMax, data.robot_pos[1] + margin);
        }
        
        const layout = {
            title: {
                text: '激光雷达扫描',
                font: { color: '#00d4ff', size: 14 }
            },
            xaxis: {
                title: 'X坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [xMin, xMax],
                scaleanchor: "y",
                scaleratio: 1,
                constrain: 'domain'
            },
            yaxis: {
                title: 'Y坐标',
                color: '#b8c5d6',
                gridcolor: '#3a4453',
                zerolinecolor: '#3a4453',
                range: [yMin, yMax],
                constrain: 'domain'
            },
            plot_bgcolor: '#252b3d',
            paper_bgcolor: '#252b3d',
            margin: { l: 50, r: 50, t: 50, b: 50 },
            showlegend: true,
            legend: {
                font: { color: '#b8c5d6' },
                bgcolor: 'rgba(37, 43, 61, 0.8)'
            },
            autosize: true
        };
        
        Plotly.react('radarChart', traces, layout);
        
        // 强制调整大小以确保内容可见
        Plotly.Plots.resize('radarChart');
    }
    
    updateStatusInfo(status) {
        // 更新状态信息面板
        document.getElementById('robotPosition').textContent = 
            `(${status.position[0].toFixed(2)}, ${status.position[1].toFixed(2)})`;
        document.getElementById('robotSteps').textContent = status.steps;
        document.getElementById('robotStatus').textContent = status.status;
        document.getElementById('frontiersCount').textContent = status.frontiers_count;
        document.getElementById('coverage').textContent = `${status.coverage.toFixed(1)}%`;
        document.getElementById('discoveredExits').textContent = status.discovered_exits;
        document.getElementById('reachedExits').textContent = status.reached_exits;
        
        // 更新路径长度信息
        if (status.path_length > 0) {
            document.getElementById('pathLength').textContent = 
                `${status.path_length.toFixed(3)} 单位`;
        } else {
            document.getElementById('pathLength').textContent = '--';
        }
    }
    
    updateProgress(coverage) {
        const progressFill = document.getElementById('progressFill');
        const progressText = document.getElementById('progressText');
        
        progressFill.style.width = `${coverage}%`;
        progressText.textContent = `探索进度: ${coverage.toFixed(1)}%`;
    }
    
    updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connectionStatus');
        const icon = statusElement.querySelector('i');
        const text = statusElement.querySelector('span');
        
        if (connected) {
            statusElement.className = 'connection-status connected';
            icon.style.color = '#00ff88';
            text.textContent = '已连接';
        } else {
            statusElement.className = 'connection-status disconnected';
            icon.style.color = '#ff3366';
            text.textContent = '已断开';
        }
    }
    
    updateSimulationStatus(running) {
        const startBtn = document.getElementById('startBtn');
        const stopBtn = document.getElementById('stopBtn');
        
        if (running) {
            startBtn.disabled = true;
            stopBtn.disabled = false;
            startBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> 运行中...';
        } else {
            startBtn.disabled = false;
            stopBtn.disabled = true;
            startBtn.innerHTML = '<i class="fas fa-play"></i> 开始仿真';
        }
    }
    
    clearStatusInfo() {
        const fields = [
            'robotPosition', 'robotSteps', 'robotStatus',
            'frontiersCount', 'coverage', 'discoveredExits',
            'reachedExits', 'pathLength'
        ];
        
        fields.forEach(field => {
            const element = document.getElementById(field);
            if (element) {
                element.textContent = '--';
            }
        });
        
        // 重置覆盖率显示
        document.getElementById('coverage').textContent = '--%';
    }
    
    addLog(message, type = 'info') {
        const logContainer = document.getElementById('logContainer');
        const logItem = document.createElement('div');
        logItem.className = `log-item ${type}`;
        
        const timestamp = new Date().toLocaleTimeString();
        logItem.textContent = `[${timestamp}] ${message}`;
        
        logContainer.appendChild(logItem);
        
        // 自动滚动到底部
        logContainer.scrollTop = logContainer.scrollHeight;
        
        // 限制日志条数
        const logItems = logContainer.querySelectorAll('.log-item');
        if (logItems.length > 100) {
            logItems[0].remove();
        }
    }
    
    clearLog() {
        const logContainer = document.getElementById('logContainer');
        logContainer.innerHTML = '<div class="log-item">系统已就绪，等待开始仿真...</div>';
    }
    
    showLoadingOverlay() {
        document.getElementById('loadingOverlay').style.display = 'block';
    }
    
    hideLoadingOverlay() {
        document.getElementById('loadingOverlay').style.display = 'none';
    }
    
    showModal(title, content) {
        const modal = document.getElementById('infoModal');
        const modalBody = document.getElementById('modalBody');
        
        modal.querySelector('h3').textContent = title;
        modalBody.innerHTML = content;
        modal.style.display = 'block';
    }
    
    hideModal() {
        document.getElementById('infoModal').style.display = 'none';
    }
    
    resizeCharts() {
        // 重新布局所有图表
        setTimeout(() => {
            Plotly.Plots.resize('trueMazeChart');
            Plotly.Plots.resize('slamMapChart');
            Plotly.Plots.resize('frontierChart');
            Plotly.Plots.resize('radarChart');
        }, 100);
    }
}

// 当页面加载完成时初始化应用
document.addEventListener('DOMContentLoaded', () => {
    window.app = new MazeSLAMApp();
});

// 全局错误处理
window.addEventListener('error', (e) => {
    console.error('全局错误:', e.error);
    if (window.app) {
        window.app.addLog('系统错误: ' + e.error.message, 'error');
    }
});

// 监听WebSocket错误
window.addEventListener('beforeunload', () => {
    if (window.app && window.app.socket) {
        window.app.socket.disconnect();
    }
}); 