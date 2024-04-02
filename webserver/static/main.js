// Download button logic
// const downloadButton = document.getElementById('downloadButton');
// const generatedCode = `{{ completion|e }}`; // Escaping completion text
// var promptTextarea = document.getElementById('promptTextarea');
// var promptEditor = CodeMirror.fromTextArea(promptTextarea, {
//     mode: 'python',
//     theme: 'default',
//     lineNumbers: true
// });

// downloadButton.addEventListener('click', () => {
//     text = promptEditor.getValue() + document.getElementById('generatedCode').innerText
//     const blob = new Blob([text], { type: 'text/plain' });
//     const url = URL.createObjectURL(blob);
//     downloadButton.href = url;
// });

// function handleFileUpload(input) {
// const file = input.files[0];
// if (file) {
//     const reader = new FileReader();
//     reader.onload = function(e) {
//         // document.getElementById('promptTextarea').value = e.target.result;
//         promptEditor.setValue(e.target.result);
//     };
//     reader.readAsText(file);
//     }
// }

$(document).ready(function() {
    $("#generate-button").click(function() {
        event.preventDefault();
        $.ajax({
            type: "POST",
            url: "/generate",
            dataType: "json",
            contentType: "application/json; charset=utf-8",    
            data: JSON.stringify(promptEditor.getValue()),
            success: function(data) {
                // console.log(data.completion)
                $("#generatedCode").text(data.completion);
                var codeElement = document.getElementById('generatedCode');
                // Add code mirror class for coloring (default is the theme)
                codeElement.classList.add( 'cm-s-default' );
                var code = codeElement.innerText;

                codeElement.innerHTML = "";

                CodeMirror.runMode(
                code,
                'python',
                codeElement
                );

            }
        });
    });

    $("#configure-form").submit(function(event) {
        event.preventDefault();

        var formData = {
            moveconf: $("input[name='moveconf']:checked").val(),
            graspconf: $("input[name='graspconf']:checked").val(),
            llmconf: $("input[name='llmconf']:checked").val()
        };
        $("#connect-status").text("Configuring & Connecting");
        $("#connect-status").css("color", "gray");

        $.ajax({
            type: "POST",
            url: "/connect",
            data: JSON.stringify(formData),
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            success: function(data) {
                console.log("Success:", data);
                var connectStatus = data.connected;
                if (connectStatus) {
                    console.log("Connected:", connectStatus)
                    $("#connect-status").text("Connected");
                    $("#connect-status").css("color", "green");
                }
            },
            error: function(jqXHR, textStatus, errorThrown) {
                console.log("Error:", textStatus, errorThrown);
            }
        });
    });

    $("#send-button").click(function() {
        sendMessage();
    });

    $("#clear-button").click(function() {
        $("#chat-window").empty(); // Clear chat window
    });

    $("#user-input").keypress(function(event) {
        if (event.which == 13) { // Enter key
            sendMessage();
        }
    });

    function sendMessage() {
        var message = $("#user-input").val();
        if (message === "") {
            return;
        }
        $("#chat-window").append("<div> <span class='you-label'>You: </span> " + message + "</div> <hr>");
        $("#user-input").val(""); // Clear input field after sending
        $.ajax({
            type: "POST",
            url: "/chat",
            data: JSON.stringify({ message: message }),
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            success: function(data) {
                console.log("Response:", data);
                for (var i = 0; i < data.messages.length; i++) {
                    // $("#chat-window").append("<div><span class='llm-label'> LLM:</span> " + data.messages[i] + "</div>");
                    if (data.messages[i].type === "text") {
                        $("#chat-window").append("<div class='llm-label'> LLM:</div> <div>" + data.messages[i].content + "</div>");
                    } else if (data.messages[i].type === "image") {
                        $("#chat-window").append("<div class='llm-label'> LLM:</div> <div> <img src='data:image/jpg;base64," + data.messages[i].content + "'></div>");
                    } else if (data.messages[i].type === "code") {
                        $("#chat-window").append("<div class='llm-label'> LLM:</div> <div> <pre><code class='python'>" + data.messages[i].content + "</code></pre></div>");
                        hljs.highlightAll();
                    }
                }
                $("#chat-window").append("<hr>");
            },
            error: function(jqXHR, textStatus, errorThrown) {
                console.log("Error:", textStatus, errorThrown);
            }
        });
    }

    $("#robot-execute").click(function() {
        $.ajax({
            type: "POST",
            url: "/execute",
            success: function(data) {
                console.log("Execute:", data);
            }
        });
    });

    $("#robot-home").click(function() {
        $.ajax({
            type: "POST",
            url: "/home",
            success: function(data) {
                console.log("Home:", data);
            }
        });
    });

    $("#robot-move").click(function() {
        $.ajax({
            type: "POST",
            url: "/move",
            success: function(data) {
                console.log("Move:", data);
            }
        });
    });

    $("#robot-grasp").click(function() {
        $.ajax({
            type: "POST",
            url: "/grasp",
            success: function(data) {
                console.log("Grasp:", data);
            }
        });
    });

    $("#robot-release").click(function() {
        $.ajax({
            type: "POST",
            url: "/release",
            success: function(data) {
                console.log("Release:", data);
            }
        });
    });
});