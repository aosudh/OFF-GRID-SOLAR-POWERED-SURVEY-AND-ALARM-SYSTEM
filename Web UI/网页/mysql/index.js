const mysql = require("mysql");
const connection = mysql.createConnection({
  host: "124.70.167.246",
  user: "test1",
  password: "123456",
  database: "test1",
});

try {
  connection.connect();
  console.log("mysql connect success");
}catch (e) {
  console.log("mysql connect fail");
}

function execSQL(sql) {
  return new Promise((resolve, reject) => {
    connection.query(sql, (err, result) => {
      if (err) {
        reject(err);
      } else {
        resolve(result);
      }
    });
  });
}

module.exports = { execSQL };
