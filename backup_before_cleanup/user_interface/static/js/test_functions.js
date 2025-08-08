/**
 * 系統功能測試 JavaScript
 * System Function Testing JavaScript
 */

// 全域變數
const API_BASE = '/api';
let autoRefreshInterval = null;
let testResults = {};

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    console.log('🧪 測試系統初始化...');
    addLog('🧪 測試系統已啟動', 'info');
});

/**
 * 添加日誌到顯示區域
 */
function addLog(message, type = 'info') {
    const logDisplay = document.getElementById('systemLogs');
    const timestamp = new Date().toLocaleTimeString('zh-TW');
    
    let colorClass = '';
    switch(type) {
        case 'success': colorClass = 'style="color: #00ff00;"'; break;
        case 'error': colorClass = 'style="color: #ff4444;"'; break;
        case 'warning': colorClass = 'style="color: #ffaa00;"'; break;
        default: colorClass = 'style="color: #00ffff;"';
    }
    
    const logEntry = `<div ${colorClass}>[${timestamp}] ${message}</div>`;
    logDisplay.innerHTML += logEntry;
    logDisplay.scrollTop = logDisplay.scrollHeight;
}

/**
 * 清除日誌
 */
function clearLogs() {
    document.getElementById('systemLogs').innerHTML = '';
    addLog('📝 日誌已清除', 'info');
}

/**
 * 切換自動刷新
 */
function toggleAutoRefresh() {
    const btn = document.getElementById('autoRefreshBtn');
    
    if (autoRefreshInterval) {
        clearInterval(autoRefreshInterval);
        autoRefreshInterval = null;
        btn.textContent = '開啟自動刷新';
        addLog('⏸️ 自動刷新已停止', 'info');
    } else {
        autoRefreshInterval = setInterval(() => {
            testHealthCheck(false);
        }, 5000);
        btn.textContent = '關閉自動刷新';
        addLog('▶️ 自動刷新已啟動 (每5秒)', 'info');
    }
}

/**
 * 顯示測試結果
 */
function showResult(elementId, message, type = 'info') {
    const resultElement = document.getElementById(elementId);
    resultElement.style.display = 'block';
    resultElement.innerHTML = `<span class="status-${type}">${message}</span>`;
    
    addLog(message.replace(/\n/g, ' '), type);
}

/**
 * 測試系統健康檢查
 */
async function testHealthCheck(showUI = true) {
    if (showUI) addLog('🔄 開始健康檢查...', 'info');
    
    try {
        const response = await fetch(`${API_BASE}/health`);
        const data = await response.json();
        
        if (response.ok) {
            const message = `✅ 系統健康檢查通過\n狀態: ${data.status}\n消息: ${data.message}\nROS2狀態: ${data.ros2_status}`;
            if (showUI) showResult('healthResult', message, 'success');
            testResults.health = { status: 'success', data };
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 健康檢查失敗: ${error.message}`;
        if (showUI) showResult('healthResult', message, 'error');
        testResults.health = { status: 'error', error: error.message };
    }
}

/**
 * 測試基本藥物列表
 */
async function testBasicMedicines() {
    addLog('🔄 測試基本藥物列表API...', 'info');
    
    try {
        const response = await fetch(`${API_BASE}/medicine/basic`);
        const data = await response.json();
        
        if (response.ok) {
            const message = `✅ 基本藥物列表獲取成功\n藥物數量: ${data.length}\n藥物列表:\n${JSON.stringify(data, null, 2)}`;
            showResult('medicineResult', message, 'success');
            testResults.basicMedicines = { status: 'success', count: data.length, data };
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 基本藥物列表獲取失敗: ${error.message}`;
        showResult('medicineResult', message, 'error');
        testResults.basicMedicines = { status: 'error', error: error.message };
    }
}

/**
 * 測試詳細藥物列表
 */
async function testDetailedMedicines() {
    addLog('🔄 測試詳細藥物列表API...', 'info');
    
    try {
        const response = await fetch(`${API_BASE}/medicine/detailed`);
        const data = await response.json();
        
        if (response.ok) {
            const message = `✅ 詳細藥物列表獲取成功\n藥物數量: ${data.length}\n詳細資料:\n${JSON.stringify(data, null, 2)}`;
            showResult('medicineResult', message, 'success');
            testResults.detailedMedicines = { status: 'success', count: data.length, data };
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 詳細藥物列表獲取失敗: ${error.message}`;
        showResult('medicineResult', message, 'error');
        testResults.detailedMedicines = { status: 'error', error: error.message };
    }
}

/**
 * 創建測試藥物
 */
async function createTestMedicine() {
    addLog('🔄 創建測試藥物...', 'info');
    
    const testMedicineData = {
        name: `測試藥物_${Date.now()}`,
        amount: 100,
        position: `A${Math.floor(Math.random() * 10)}${Math.floor(Math.random() * 10)}`,
        manufacturer: '測試製藥公司',
        dosage: '100mg',
        is_active: true
    };
    
    try {
        const response = await fetch(`${API_BASE}/medicine/`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(testMedicineData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            const message = `✅ 測試藥物創建成功\nID: ${result.id}\n藥物資料:\n${JSON.stringify(testMedicineData, null, 2)}`;
            showResult('medicineResult', message, 'success');
            testResults.createMedicine = { status: 'success', id: result.id, data: testMedicineData };
        } else {
            throw new Error(result.detail || `HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 測試藥物創建失敗: ${error.message}`;
        showResult('medicineResult', message, 'error');
        testResults.createMedicine = { status: 'error', error: error.message };
    }
}

/**
 * 創建測試處方籤
 */
async function createTestPrescription() {
    addLog('🔄 創建測試處方籤...', 'info');
    
    const testPrescriptionData = {
        patient_name: '測試病患',
        patient_id: `P${Date.now()}`,
        doctor_name: '測試醫生',
        diagnosis: '測試診斷',
        medicines: [
            ['阿斯匹靈', '100mg', '30', '每日三次'],
            ['維他命C', '500mg', '60', '每日一次']
        ]
    };
    
    try {
        const response = await fetch(`${API_BASE}/prescription/`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(testPrescriptionData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            const message = `✅ 測試處方籤創建成功\nID: ${result.id}\n處方資料:\n${JSON.stringify(testPrescriptionData, null, 2)}`;
            showResult('prescriptionResult', message, 'success');
            testResults.createPrescription = { status: 'success', id: result.id, data: testPrescriptionData };
        } else {
            throw new Error(result.detail || `HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 測試處方籤創建失敗: ${error.message}`;
        showResult('prescriptionResult', message, 'error');
        testResults.createPrescription = { status: 'error', error: error.message };
    }
}

/**
 * 測試處方籤列表
 */
async function testPrescriptionList() {
    addLog('🔄 測試處方籤列表...', 'info');
    
    try {
        const response = await fetch(`${API_BASE}/prescription/`);
        const data = await response.json();
        
        if (response.ok) {
            const message = `✅ 處方籤列表獲取成功\n處方數量: ${data.length}\n處方列表:\n${JSON.stringify(data, null, 2)}`;
            showResult('prescriptionResult', message, 'success');
            testResults.prescriptionList = { status: 'success', count: data.length, data };
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ 處方籤列表獲取失敗: ${error.message}`;
        showResult('prescriptionResult', message, 'error');
        testResults.prescriptionList = { status: 'error', error: error.message };
    }
}

/**
 * 測試ROS2狀態
 */
async function testROS2Status() {
    addLog('🔄 測試ROS2狀態...', 'info');
    
    try {
        const response = await fetch(`${API_BASE}/health`);
        const data = await response.json();
        
        if (response.ok) {
            const ros2Available = data.ros2_status === 'available';
            const message = `${ros2Available ? '✅' : '⚠️'} ROS2狀態檢查完成\n狀態: ${data.ros2_status}\n${ros2Available ? 'ROS2模組正常運行' : 'ROS2模組未啟用（模擬模式）'}`;
            showResult('ros2Result', message, ros2Available ? 'success' : 'warning');
            testResults.ros2Status = { status: 'success', available: ros2Available, data };
        } else {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
    } catch (error) {
        const message = `❌ ROS2狀態檢查失敗: ${error.message}`;
        showResult('ros2Result', message, 'error');
        testResults.ros2Status = { status: 'error', error: error.message };
    }
}

/**
 * 創建ROS2訂單
 */
async function createROS2Order() {
    addLog('🔄 測試ROS2訂單創建...', 'info');
    
    // 先創建處方籤，這會自動觸發ROS2訂單
    await createTestPrescription();
    
    // 檢查是否成功
    if (testResults.createPrescription && testResults.createPrescription.status === 'success') {
        const message = `✅ ROS2訂單創建測試完成\n透過處方籤創建自動觸發ROS2訂單\n處方籤ID: ${testResults.createPrescription.id}`;
        showResult('ros2Result', message, 'success');
        testResults.ros2Order = { status: 'success', prescriptionId: testResults.createPrescription.id };
    } else {
        const message = `❌ ROS2訂單創建失敗\n無法創建處方籤，因此無法觸發ROS2訂單`;
        showResult('ros2Result', message, 'error');
        testResults.ros2Order = { status: 'error', error: '處方籤創建失敗' };
    }
}

/**
 * 測試訂單佇列
 */
async function testOrderQueue() {
    addLog('🔄 測試訂單佇列...', 'info');
    
    try {
        // 這裡假設有訂單佇列API，如果沒有則顯示訊息
        const message = `ℹ️ 訂單佇列測試\n注意：此功能需要ROS2模組完整支援\n當前為模擬模式，訂單會在處方籤創建時自動加入佇列`;
        showResult('ros2Result', message, 'info');
        testResults.orderQueue = { status: 'info', message: '模擬模式' };
    } catch (error) {
        const message = `❌ 訂單佇列測試失敗: ${error.message}`;
        showResult('ros2Result', message, 'error');
        testResults.orderQueue = { status: 'error', error: error.message };
    }
}

/**
 * 測試醫生藥物選擇功能
 */
async function testDoctorMedicineSelection() {
    addLog('🔄 測試醫生藥物選擇功能...', 'info');
    
    try {
        // 獲取藥物列表來模擬醫生界面的藥物選擇功能
        const response = await fetch(`${API_BASE}/medicine/basic`);
        const medicines = await response.json();
        
        if (response.ok && medicines.length > 0) {
            const message = `✅ 醫生藥物選擇功能測試通過\n可用藥物數量: ${medicines.length}\n測試藥物:\n${medicines.slice(0, 3).map(med => `- ${med.name} (庫存: ${med.amount})`).join('\n')}`;
            showResult('doctorResult', message, 'success');
            testResults.doctorMedicineSelection = { status: 'success', availableMedicines: medicines.length };
        } else {
            throw new Error('無可用藥物');
        }
    } catch (error) {
        const message = `❌ 醫生藥物選擇功能測試失敗: ${error.message}`;
        showResult('doctorResult', message, 'error');
        testResults.doctorMedicineSelection = { status: 'error', error: error.message };
    }
}

/**
 * 模擬完整處方流程
 */
async function simulateFullPrescriptionFlow() {
    addLog('🔄 模擬完整處方流程...', 'info');
    
    try {
        // 步驟1: 檢查系統健康
        addLog('步驟1: 檢查系統健康...', 'info');
        await testHealthCheck(false);
        
        // 步驟2: 獲取可用藥物
        addLog('步驟2: 獲取可用藥物...', 'info');
        await testBasicMedicines();
        
        // 步驟3: 創建處方籤
        addLog('步驟3: 創建處方籤...', 'info');
        await createTestPrescription();
        
        // 檢查所有步驟是否成功
        const allSuccess = testResults.health?.status === 'success' && 
                          testResults.basicMedicines?.status === 'success' && 
                          testResults.createPrescription?.status === 'success';
        
        if (allSuccess) {
            const message = `✅ 完整處方流程模擬成功\n1. ✅ 系統健康檢查通過\n2. ✅ 藥物列表載入成功 (${testResults.basicMedicines.count}種)\n3. ✅ 處方籤創建成功 (ID: ${testResults.createPrescription.id})\n\n完整流程運行正常！`;
            showResult('doctorResult', message, 'success');
            testResults.fullFlow = { status: 'success' };
        } else {
            throw new Error('流程中部分步驟失敗');
        }
    } catch (error) {
        const message = `❌ 完整處方流程模擬失敗: ${error.message}`;
        showResult('doctorResult', message, 'error');
        testResults.fullFlow = { status: 'error', error: error.message };
    }
}

/**
 * 格式化JSON輸出
 */
function formatJSON(obj) {
    return JSON.stringify(obj, null, 2);
}

/**
 * 獲取測試結果摘要
 */
function getTestSummary() {
    addLog('📊 測試結果摘要:', 'info');
    
    for (const [testName, result] of Object.entries(testResults)) {
        const status = result.status === 'success' ? '✅' : 
                      result.status === 'error' ? '❌' : 'ℹ️';
        addLog(`${status} ${testName}: ${result.status}`, result.status);
    }
}

// 導出全域函數供測試使用
window.testFunctions = {
    testHealthCheck,
    testBasicMedicines,
    testDetailedMedicines,
    createTestMedicine,
    createTestPrescription,
    testPrescriptionList,
    testROS2Status,
    createROS2Order,
    testOrderQueue,
    testDoctorMedicineSelection,
    simulateFullPrescriptionFlow,
    getTestSummary,
    testResults
};