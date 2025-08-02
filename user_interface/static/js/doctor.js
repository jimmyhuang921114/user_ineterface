document.addEventListener("DOMContentLoaded", () => {
    const container = document.getElementById("excelTable");
    
    if (container) {
        const hot = new Handsontable(container, {
            data: [
                ["", "", "", "", ""],
                ["", "", "", "", ""],
                ["", "", "", "", ""]
            ],
            colHeaders: ["藥物名稱", "劑量", "頻率", "療程", "特殊說明"],
            columns: [
                { type: 'text', placeholder: '請輸入藥物名稱' },
                { type: 'text', placeholder: '例如: 100mg' },
                { type: 'text', placeholder: '例如: 每日三次' },
                { type: 'text', placeholder: '例如: 7天' },
                { type: 'text', placeholder: '特殊用藥說明' }
            ],
            rowHeaders: true,
            minSpareRows: 1,
            stretchH: 'all',
            height: 300,
            licenseKey: 'non-commercial-and-evaluation'
        });

        // 處方籤表單提交處理
        const prescriptionForm = document.getElementById("prescriptionForm");
        if (prescriptionForm) {
            prescriptionForm.addEventListener("submit", function (e) {
                e.preventDefault();
                
                const name = document.getElementById("PatientName")?.value.trim();
                const id = document.getElementById("id_number")?.value.trim();
                const sex = document.querySelector('input[name="sex"]:checked')?.value || "Unknown";
                
                const prescription = hot.getData().filter(row => 
                    row.some(cell => cell !== null && cell.toString().trim() !== "")
                );

                if (!name || !id || prescription.length === 0) {
                    alert("請填寫完整的患者資訊和處方藥物");
                    return;
                }

                const patient = {
                    name,
                    id,
                    sex,
                    created_at: new Date().toLocaleString(),
                    prescription,
                    picked_state: "待處理",
                    collected_state: "未領取"
                };

                console.log("提交的患者資料:", patient);
                alert("處方籤已成功開立\n請查看 console 確認資料");

                // 重置表單
                this.reset();
                hot.loadData([
                    ["", "", "", "", ""],
                    ["", "", "", "", ""],
                    ["", "", "", "", ""]
                ]);
            });
        }
    }
});

// 如果在doctor.html頁面，添加標籤頁切換功能
if (window.location.pathname.includes('doctor.html')) {
    // 標籤頁切換功能
    window.switchTab = function(tabName) {
        console.log('切換到標籤頁:', tabName);
        
        // 隱藏所有標籤頁內容
        document.querySelectorAll('.tab-content').forEach(content => {
            content.classList.remove('active');
        });

        // 移除所有按鈕的 active 狀態
        document.querySelectorAll('.tab-button').forEach(button => {
            button.classList.remove('active');
        });

        // 顯示選中的標籤頁
        const targetTab = document.getElementById(tabName);
        if (targetTab) {
            targetTab.classList.add('active');
        } else {
            console.error('找不到標籤頁:', tabName);
        }
        
        // 設置按鈕為 active 狀態
        if (event && event.target) {
            event.target.classList.add('active');
        } else {
            // 備用方案: 手動查找對應按鈕
            const buttons = document.querySelectorAll('.tab-button');
            buttons.forEach((button, index) => {
                if ((tabName === 'basic' && index === 0) ||
                    (tabName === 'detailed' && index === 1) ||
                    (tabName === 'prescription' && index === 2)) {
                    button.classList.add('active');
                }
            });
        }

        // 處方籤標籤頁需要初始化表格
        if (tabName === 'prescription') {
            setTimeout(() => {
                if (typeof initPrescriptionTable === 'function') {
                    initPrescriptionTable();
                }
            }, 100);
        }
    };

    // 藥物選項載入功能
    window.loadMedicineOptions = async function() {
        console.log('載入藥物選項...');
        try {
            const response = await fetch('/api/medicine/');
            if (response.ok) {
                const medicines = await response.json();
                const select = document.getElementById('detailedMedicineName');
                if (select) {
                    select.innerHTML = '<option value="">-- 請選擇藥物 --</option>';
                    medicines.forEach(medicine => {
                        const option = document.createElement('option');
                        option.value = medicine.name;
                        option.textContent = `${medicine.name} (位置: ${medicine.position})`;
                        select.appendChild(option);
                    });
                    console.log(`載入了 ${medicines.length} 種藥物`);
                }
            }
        } catch (error) {
            console.error('載入藥物選項失敗:', error);
        }
    };

    // 清空表單功能
    window.clearBasicForm = function() {
        const form = document.getElementById('basicMedicineForm');
        if (form) {
            form.reset();
        }
    };

    window.clearDetailedForm = function() {
        const form = document.getElementById('detailedMedicineForm');
        if (form) {
            form.reset();
        }
    };

    window.clearPrescriptionForm = function() {
        const form = document.getElementById('prescriptionForm');
        if (form) {
            form.reset();
        }
        // 重設日期為今天
        const dateInput = document.getElementById('prescriptionDate');
        if (dateInput) {
            dateInput.value = new Date().toISOString().split('T')[0];
        }
    };
}